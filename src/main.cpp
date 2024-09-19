#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <SimpleDHT.h>
#include <Adafruit_I2CDevice.h>


#define DHT_PIN 23   // Pino em que o DHT11 está conectado

SimpleDHT11 dht11;  // Inicializa o sensor DHT11

// Definição dos pinos LoRa
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND 433E6  // Frequência de operação do LoRa

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);
  while (!Serial);

  // Inicializa LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI00);

  if (!LoRa.begin(BAND)) {
    Serial.println("Erro ao inicializar o módulo LoRa.");
    while (1);
  }

  Serial.println("LoRa inicializado com sucesso.");
}

void loop() {
  // Variáveis para armazenar temperatura e umidade
  byte temperature = 0;
  byte humidity = 0;

  // Lê os dados do sensor DHT11
  int err = dht11.read(DHT_PIN, &temperature, &humidity, NULL);

  if (err != SimpleDHTErrSuccess) {
    Serial.print("Falha ao ler o sensor DHT11, erro: ");
    Serial.println(err);
    return;
  }

  // Exibe os valores na serial
  Serial.print("Temperatura: ");
  Serial.print((int)temperature);
  Serial.print(" *C, Umidade: ");
  Serial.print((int)humidity);
  Serial.println(" %");

  // Monta o pacote de dados
  String packet = "Temperatura: " + String((int)temperature) + " *C, Umidade: " + String((int)humidity) + " %";

  // Envia o pacote via LoRa
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  Serial.println("Dados enviados via LoRa.");

  // Aguarda 2 segundos antes da próxima leitura
  delay(2000);
}



