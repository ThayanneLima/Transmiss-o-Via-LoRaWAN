#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>

#define CFG_au915 1

#define DHTPIN 4      // Pino onde o sensor DHT está conectado
#define DHTTYPE DHT11 // Tipo de sensor DHT: DHT11

DHT dht(DHTPIN, DHTTYPE);

// Chaves LoRaWAN (substitua pelos seus valores)
static const PROGMEM u1_t NWKSKEY[16] = {0xA0, 0x1B, 0xA7, 0x6C, 0xA3, 0x14, 0xC4, 0xA4, 0x5D, 0x9F, 0xE1, 0x0E, 0xE5, 0x63, 0x15, 0x0F};
static const PROGMEM u1_t APPSKEY[16] = {0x0C, 0xD0, 0xDF, 0x1D, 0xAE, 0x72, 0x21, 0xC8, 0xC7, 0xF1, 0xA9, 0xBF, 0xAE, 0xBE, 0x89, 0x59};
static const u4_t DEVADDR = 0x260D32EF; // Device Address 0x26, 0x0D, 0x32, 0xEF

// Declaração da variável sendjob
osjob_t sendjob;

//hahaha
// Definição dos pinos para o LoRa
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}
};

void os_getArtEui (u1_t* buf) {}
void os_getDevEui (u1_t* buf) {}
void os_getDevKey (u1_t* buf) {}

void do_send(osjob_t* j) {
    float temperature = dht.readTemperature();
    if (isnan(temperature)) {
        Serial.println("Falha ao ler o sensor DHT!");
        return;
    }

    // Imprimir a temperatura em graus Celsius
    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Preparar payload
    uint8_t payload[2];
    int16_t temp = temperature * 100;
    payload[0] = temp >> 8;
    payload[1] = temp;

    // Enviar
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println("Temperatura enviada!");
}

void onEvent(ev_t ev) {
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println("Transmissão completa!");
            // Agendar o próximo envio a cada 2 segundos
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    dht.begin();

    // Inicialize a biblioteca LMIC com as chaves configuradas
    LMIC_setSession(0x1, DEVADDR, const_cast<u1_t*>(NWKSKEY), const_cast<u1_t*>(APPSKEY));

    // Inicializar LMIC
    os_init();
    LMIC_reset();

    // Definir canais
    LMIC_selectSubBand(1);
    LMIC_setDrTxpow(DR_SF7, 14); // Defina a taxa de dados e a potência de transmissão

    // Agendar primeiro envio
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
