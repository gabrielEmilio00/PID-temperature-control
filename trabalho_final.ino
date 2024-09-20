#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <MQTT.h>

unsigned long tempoAtual = 0; 
unsigned long tempoAnterior = 0;

float SP = 35; // Setpoint em °C

// Pinos utilizados
const uint8_t pinAquecedor = 21;
const uint8_t pinResfriador = 19;
const uint8_t pinSensor = 5;

char* topicos[] = {"setpoint", "bancada3/kp", "bancada3/ki", "bancada3/kd"};

// Constantes do controlador
float Kp = 20, Ki = 0.1, Kd = 0;
// Variaveis de ação do controlador
float P, I, D; 

// Número de repetição do cursor circular
const uint8_t N = 4;
//Cursor circular
float erros[N];
uint8_t cursor_erros = 0;

// Somatorio dos erros no calculo da integral
float somatoria;

// valor do ajuste
float alpha = 0;

WiFiClient net;
MQTTClient client;
// Configurando bibliotecas OneWire e DallasTemperature
OneWire oneWire(pinSensor);                                                                                                                                    
DallasTemperature sensors(&oneWire);

const char ssid[] = "LabMicroPrivNet";
const char pass[] = "eaibeleza";

void connect() {
  Serial.print("Conectando no Wifi");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConexão realizada com sucesso");
  Serial.println("Conectando ao servidor MQTT...");
  while(!client.connect("Bancada3", "", "")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("Inscrevendo nos tópicos do MQTT...");

  for (auto i = 0; i < 4; i++) {
    client.subscribe(topicos[i]);
  }

  Serial.println("\nInscrição concluída");
  delay(100);
}

void messageReceived(String &topic, String &payload) {
  Serial.println("Incoming: " + topic + " - " + payload);

  if (topic == "setpoint") {
    SP = payload.toFloat();
    // somatoria = 0;
  }
  else if (topic == "bancada3/kp") {
    Kp = payload.toFloat();
  } 
  else if (topic == "bancada3/ki") {
    Ki = payload.toFloat();
    if (Ki == 0) {
      somatoria = 0;
    }
  }
  else { // kd
    Kd = payload.toFloat();
  }
  
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {
  Serial.begin(115200);

  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  // Configurando WiFi e MQTT
  WiFi.begin(ssid, pass);
  client.begin("192.168.0.101", net);
  client.onMessage(messageReceived);

  connect();


  // Configurando pinos
  ledcAttach(pinAquecedor, 10000, 12);
  ledcAttach(pinResfriador, 10000, 12);

  // Iniciando sensor de temperatura
  sensors.begin();
}

void loop() {
  // Atualizar MQTT
  client.loop();
  delay(20);

  if (!client.connected()) {
    connect();
  }

  tempoAtual = millis();

  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0); // Obter temperatura do sensor em °C

  if (tempC != DEVICE_DISCONNECTED_C) {
    float erro = SP - tempC; // Calcular erro
    erros[cursor_erros] = erro;


    // Calculo da média (derivada)
    auto proximo = (cursor_erros + 1) % N;

    float erroAtual = erros[cursor_erros];
    float erroMaisAntigo = erros[proximo];

    float media = (erroAtual - erroMaisAntigo) / N;

    cursor_erros = proximo;

    // Calculo da Somatoria dos erros (integral)
    somatoria += erro*Ki;
    

    P = Kp*erro; // Ação Proporcional
    I = somatoria; // Ação Integral
    D = Kd*media; // Ação Derivativa

    alpha = P + I + D; // MV

    alpha = constrain(alpha, -100, 100);

    alpha = fmap(alpha, -100, 100, 0, 100);

    if (alpha <= 49) {
      auto duty_resfriador = fmap(alpha, 0, 49, 4095, 2000);
      ledcWrite(pinResfriador, duty_resfriador);
      ledcWrite(pinAquecedor, 0);
    }
    else if (alpha >= 51) {
      auto duty_aquecedor = fmap(alpha, 51, 100, 2000, 4095);
      ledcWrite(pinAquecedor, duty_aquecedor);
      ledcWrite(pinResfriador, 0);
    }
    else {
      ledcWrite(pinAquecedor, 0);
      ledcWrite(pinResfriador, 0);
    }

    char temperatura[80];
    sprintf(temperatura, "%f", tempC);
    client.publish("bancada3/temperatura", temperatura);
  
    if ((tempoAtual - tempoAnterior) >= 300) {
 
      Serial.print("Temperatura: ");
      Serial.println(temperatura);
      
      char valorErro[30];
      sprintf(valorErro, "Erro: %06.3f", erro);
      Serial.println(valorErro);

      char constanteP[30];
      sprintf(constanteP, "P: %06.3f", P);
      Serial.println(constanteP);
      char constanteI[30];
      sprintf(constanteI, "I: %06.3f", I);
      Serial.println(constanteI);
      char constanteD[30];
      sprintf(constanteD, "D: %06.3f", D);
      Serial.println(constanteD);

      char controle[30];
      sprintf(controle, "Alpha: %06.3f", alpha);
      Serial.println(controle);

      tempoAnterior = tempoAtual;
    }
  }
  else {
    Serial.println("Erro ao obter a temperatura");
  }
}


