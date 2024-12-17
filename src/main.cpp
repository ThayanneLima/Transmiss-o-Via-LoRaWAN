#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <lmic.h>
#include <hal/hal.h>

#define  LMIC_DEBUG_LEVEL = 1 
#define CFG_au915

#define LORA_GAIN 20
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif
//objeto para o sensor BMP280:
Adafruit_BMP280 bmp;


void buildPacket(uint8_t txBuffer[9]);
void do_send(osjob_t* j);
/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
static const PROGMEM u1_t NWKSKEY[16] = {0xF9, 0xD9, 0xC0, 0xFB, 0xA0, 0x8D, 0x7D, 0x46, 0xC0, 0x30, 0xD7, 0xC8, 0x41, 0x5C, 0x5A, 0x96};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0x8B, 0x03, 0xEB, 0x14, 0xED, 0xEA, 0x42, 0x77, 0xEA, 0x19, 0x58, 0xCB, 0xD2, 0xC4, 0xEE, 0x07};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260DD8FC ; // <-- Change this address for every node!
*/

/*
// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
//cbb26990ffbccf7313672a5fa3ec20e3
static const PROGMEM u1_t NWKSKEY[16] = {0x28, 0xF6, 0xFA, 0xA2, 0x63, 0xA8, 0x61, 0x73, 0x95, 0xC3, 0xAA, 0x06, 0x3B, 0x9A, 0x41, 0xCE};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0xFE, 0x93, 0xBF, 0x1D, 0x7A, 0x50, 0x5B, 0x9A, 0x2B, 0xF2, 0x27, 0x0F, 0xD1, 0xAE, 0x62, 0xF2};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260D550F ; // <-- Change this address for every node!
*/
//config for  DHT Lorawan
static const PROGMEM u1_t NWKSKEY[16] = {0xA0, 0x1B, 0xA7, 0x6C, 0xA3, 0x14, 0xC4, 0xA4, 0x5D, 0x9F, 0xE1, 0x0E, 0xE5, 0x63, 0x15, 0x0F};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0x0C, 0xD0, 0xDF, 0x1D, 0xAE, 0x72, 0x21, 0xC8, 0xC7, 0xF1, 0xA9, 0xBF, 0xAE, 0xBE, 0x89, 0x59};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260D32EF;


int n_packet=0;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//static uint8_t mydata[4];
uint8_t txBuffer[24];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 15; // 300=5 minutos

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,   // For TTGO 14, T-Beam 23
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};


#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16



void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}
int count =5;
void do_send(osjob_t* j) {  
  // Check if there is not a current TX/RX job running
      
  //displayValues();
 
      if (LMIC.opmode & OP_TXRXPEND)
      {
        Serial.println(F("OP_TXRXPEND, not sending"));
        //LoraStatus = "OP_TXRXPEND, not sending";
      }
      else
      { 
          //led.clear();
          //led.drawString(0,0,"fake packet");
          buildPacket(txBuffer);

          n_packet++;
          LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0); //é responsável por enviar os dados no uplink para o servidor LoRaWAN
          Serial.println(F("Packet queued"));
          //LoraStatus = "Packet queued";
      }

  // Next TX is scheduled after TX_COMPLETE event.
  
}


void setupLoRaWAN()
{
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band  
 // LMIC_setupChannel(8, 994000000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  2);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);

#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF10;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF10, LORA_GAIN);


  // Start job
  do_send(&sendjob);
 
}




void setup() {
  // Inicializando os comandos no TTGO lora32 (placa usada para ensino na unioeste).
  Serial.begin(115200);
  
  Serial.println("Iniciando");

  // Inicializa o barramento I2C nos pinos GPIO 21 (SDA) e GPIO 22 (SCL)
  Wire.begin(21, 22);

  setupLoRaWAN();  

  // Inicializa o sensor BMP280
  if (!bmp.begin(0x76)) { // Endereço I2C padrão do BMP280
    Serial.println("Erro ao inicializar o BMP280!");
    while (1);
  } else {
    Serial.println("BMP280 inicializado com sucesso!");
  }

  setupLoRaWAN(); // Inicialização do LoRaWAN

  //teste sensor BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Sensor BMP280 não encontrado. Verifique as conexões!");
    while (1);
} else {
    Serial.println("Sensor BMP280 inicializado com sucesso!");
}

}

unsigned long displayTime=0;
void loop()
{
  unsigned long currentTime= millis();
  if (currentTime - displayTime > 5000) {
    Serial.println(n_packet);
    displayTime= currentTime;
  }
    
  os_runloop_once();
                                 
 /* if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));*/
}


void buildPacket(uint8_t txBuffer[9])
{
  float temperatura = bmp.readTemperature();     // Lê a temperatura em °C
  float pressao = bmp.readPressure() / 100.0;    // Lê a pressão em hPa

  // Formata os valores para enviar via LoRaWAN (em bytes)
  int16_t temp = (int16_t)(temperatura * 100);   // Converte para centésimos de grau
  uint16_t pres = (uint16_t)(pressao * 10);      // Converte para décimos de hPa

  //Formato do pacote: Os valores são colocados em um buffer de bytes (txBuffer) para envio
  txBuffer[0] = highByte(temp);
  txBuffer[1] = lowByte(temp);
  txBuffer[2] = highByte(pres);
  txBuffer[3] = lowByte(pres);

  // Preenche os bytes restantes com zeros (ou use-os para outros dados, se necessário)
  for (int i = 4; i < 9; i++) {
    txBuffer[i] = 0;
  }

  Serial.print("Temperatura: "); Serial.print(temperatura); Serial.println(" °C");
  Serial.print("Pressão: "); Serial.print(pressao); Serial.println(" hPa");
 //temp DHT
 // txBuffer[0] =0; 
 // txBuffer[1] =1;
 // txBuffer[2] =2;
 // txBuffer[3] =3;
 // txBuffer[4] =4;


}


void displayValues() {
  // tempo de espera entre cada leitura.
  // Ler o valor da temperatura e imprimir esse valor no monitor serial.
  //led.clear();
 
   // led.drawString(0,0,"pacote: ");
   // led.drawString(5,0, n_packet);    
 
}