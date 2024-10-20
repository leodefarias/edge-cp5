#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"  // Biblioteca para o sensor MAX30105

const char* default_SSID = ""; 
const char* default_PASSWORD = ""; 
const char* default_BROKER_MQTT = ""; 
const int default_BROKER_PORT = 1883;
const char* default_TOPICO_PUBLISH_HEART = "/sensor/max30105/heart";
const char* default_TOPICO_PUBLISH_SPO2 = "/sensor/max30105/spo2";  
const char* default_ID_MQTT = "esp32_max30105"; 

MAX30105 particleSensor;  // Instância do sensor
WiFiClient espClient;
PubSubClient client(espClient);

// Definição do pino do LED
const int LED_PIN = 25;

void initSerial() {
    Serial.begin(115200);
    delay(10);
}

void initWiFi() {
    Serial.println("Conectando-se ao Wi-Fi...");
    WiFi.begin(default_SSID, default_PASSWORD);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
        Serial.print(".");
        delay(1000);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConectado!");
        Serial.print("IP obtido: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFalha ao conectar ao Wi-Fi.");
    }
}

void initMQTT() {
    client.setServer(default_BROKER_MQTT, default_BROKER_PORT);
    client.setCallback(mqtt_callback);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    Serial.print("Mensagem recebida no tópico ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(msg);
}

void setup() {
    initSerial();
    initWiFi();
    initMQTT();

    // Inicializa o LED como saída
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Desliga o LED inicialmente

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("Sensor MAX30105 não encontrado!");
        while (1);  // Trava o código se o sensor não for encontrado
    }

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);  // Ativa LED vermelho
    particleSensor.setPulseAmplitudeGreen(0);   // Desativa LED verde

    Serial.println("Inicialização concluída.");
}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    if (irValue > 50000) {
        int heartRate = random(60, 100);  // Simulação
        int spo2 = random(95, 100);       

        String heartRateStr = String(heartRate);
        String spo2Str = String(spo2);

        Serial.println("Publicando dados...");
        client.publish(default_TOPICO_PUBLISH_HEART, heartRateStr.c_str());
        client.publish(default_TOPICO_PUBLISH_SPO2, spo2Str.c_str());

        Serial.print("Batimentos: ");
        Serial.println(heartRateStr);
        Serial.print("SpO2: ");
        Serial.println(spo2Str);

        // Verifica se os valores estão fora do normal e aciona o LED
        if (heartRate < 60 || heartRate > 100 || spo2 < 95) {
            digitalWrite(LED_PIN, HIGH);  // Acende o LED
            Serial.println("Valores fora do normal! LED ativado.");
        } else {
            digitalWrite(LED_PIN, LOW);   // Apaga o LED
            Serial.println("Valores normais. LED desativado.");
        }
    }

    delay(2000);  // Intervalo entre leituras
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Tentando se conectar ao Broker MQTT...");
        if (client.connect(default_ID_MQTT)) {
            Serial.println("Conectado ao Broker MQTT!");
        } else {
            Serial.print("Falha na conexão. Código: ");
            Serial.print(client.state());
            Serial.println(" Tentando novamente em 5 segundos.");
            delay(5000);
        }
    }
}
