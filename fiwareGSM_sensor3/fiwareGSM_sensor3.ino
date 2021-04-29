//Prototipo medición calidad del aire CIF - CINTEL
//Sistema para medir sensor PPM PMS5003, MQ7, MQ135 y DHT11, trasmitiendo datos a context broker Orion por GSM
//Carlos Sativa - 160421
//V1.0 - 160421
//V1.1 - Limpieza de variables - 170421
//V1.2 - Eliminación de puerto Serial serialMon, se agrega LED - 180421
//Modificaciones de Lina en la rama 2

// Configuración APN data - Información para conexión a APN - SIM card
const char apn[]      = "web.vmc.net.co"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = ""; // GPRS User
const char gprsPass[] = ""; // GPRS Password
const char simPIN[]   = ""; // SIM card PIN (leave empty, if not defined)

// LILYGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
#define I2C_SDA_2            18
#define I2C_SCL_2            19

//Constantes sensor MQ
const int ad_mq7= 34;
const int ad_mq135 = 35;
const float v_ref = 3.3;
const int Ro = 5585;
const int Ro_co = 7500;
const float factor = 100.03;
const float exponente = -3.104;
const float factor_co = 108.2;
const float exponente_co = -1.4;
float co_ppm = 0;
float co2_ppm = 0;

//LED INDICADOR DE COMUNICACIÓN
int LED_IND = 15;

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

//################# LIBRARIES ################
#include <TinyGsmClient.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "DHT.h"
 
#define DHTPIN 13 //Se cambia el PIN 4 que estaba siendo usado por el modem, por el PIN 13 310321
#define DHTTYPE DHT11

SoftwareSerial pmsSerial(18, 19); //Comunicación serial a sensor PPM
DHT dht(DHTPIN, DHTTYPE);
 
#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, Serial); //Envía por terminal serial los mismos comandos AT enviados por el módulo SIM800
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000000UL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */
    
//################ FIWARE VARIABLES ################
String FIWARE_DEVICE = "sensorCIFTest3"; //Se ajusta a sensor CIF http://18.189.192.136:1026/v2/entities/sensorCIFTest3/attrs
char FIWARE_APIKEY[] = "";
char FIWARE_SERVER[] = "18.189.192.136";
int FIWARE_PORT = 1026;
String resource = "/v2/entities/" + FIWARE_DEVICE + "/attrs";  

//################ SENSOR VARIABLES ################  
const int numsensors = 2;
String measures[numsensors][2];

//########## ESTRUCTURA DATOS - SENSOR PPM ##########  
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;

void setup() {
  pinMode(LED_IND, OUTPUT);
  digitalWrite(LED_IND, HIGH);
  Serial.begin(115200);  // Set serial monitor debugging window baud rate to 115200
  delay(200);
  dht.begin();
  delay(100);
  Serial.println(""); //Cambio de línea
  Serial.println("Sensor DHT inicializado...");
  // Set modem reset, enable, power pins
  pmsSerial.begin(9600);
  Serial.println("Inicializando sensor PPM...");
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  digitalWrite(LED_IND, LOW);
  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("V1.2 - Inicializando modem GPRS...");
  modem.restart();
  // use modem.init() if you don't need the complete restart
  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

  
  void loop() 
  {
    digitalWrite(LED_IND, HIGH);
    Serial.print("Conectandose a APN: ");
    Serial.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      Serial.println(" Fallo!");
    }
    else {
      Serial.println(" OK conexion datos");
      digitalWrite(LED_IND, LOW);
      Serial.print("Conectandose a ");
      Serial.print(FIWARE_SERVER);
      if (!client.connect(FIWARE_SERVER, FIWARE_PORT)) {
        Serial.println(" Fallo");
      }
      else {
        Serial.println(" OK");
        digitalWrite(LED_IND, HIGH);
        readSensors();
        // Making an HTTP POST request - EJEMPLO TINYGSM
        Serial.println("Ejecutando HTTP POST request...");
        //String httpRequestData = "{'temperature':{'type':'Number','value':'16.8'},'relativeHumidity':{'type':'Number','value':'59.25'}}";
        /*String httpRequestData = "{\"temperature\":{\"type\":\"Number\",\"value\":\"" + measures[0][1] + 
                                  "\"},\"relativeHumidity\":{\"type\":\"Number\",\"value\":\"" + measures[1][1] + 
                                  "\"},\"CO2\":{\"type\":\"Number\",\"value\":\"" + co2_ppm + 
                                  "\"},\"CO\":{\"type\":\"Number\",\"value\":\"" + co_ppm + 
                                  "\"}, \"PM1\":{\"type\":\"Number\",\"value\":\"" + data.particles_10um +
                                  "\"}, \"PM2.5\":{\"type\":\"Number\",\"value\":\"" + data.particles_25um +
                                  "\"}, \"PM10\":{\"type\":\"Number\",\"value\":\"" + data.particles_100um +
                                  "\"}}";*/ //Original con todas las variables de material particulado
        String httpRequestData = "{\"temperature\":{\"type\":\"Number\",\"value\":\"" + measures[0][1] + 
                                  "\"},\"relativeHumidity\":{\"type\":\"Number\",\"value\":\"" + measures[1][1] + 
                                  "\"},\"CO2\":{\"type\":\"Number\",\"value\":\"" + co2_ppm + 
                                  "\"},\"CO\":{\"type\":\"Number\",\"value\":\"" + co_ppm + 
                                  "\"}, \"PM10\":{\"type\":\"Number\",\"value\":\"" + data.particles_100um +
                                  "\"}}";      
        client.print(String("POST ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + String(FIWARE_SERVER) + String(":") + String(FIWARE_PORT) + "\r\n");
        client.println("Connection: close");
        client.println("Content-Type: application/json");
        client.print("Content-Length: ");
        client.println(httpRequestData.length());
        client.println();
        client.println(httpRequestData);
        Serial.println(httpRequestData);
        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
          while (client.available()) { // Print available data (HTTP response from server)
            char c = client.read();
            Serial.print(c);
            timeout = millis();
          }
        }
        Serial.println();
        client.stop(); // Close client and disconnect
        Serial.println(F("Servidor desconectado"));
        modem.gprsDisconnect();
        Serial.println(F("GPRS desconectado"));
            }
      }
      digitalWrite(LED_IND, LOW);
      Serial.print("Durmiendo... "); 
      esp_deep_sleep_start();
  }
   
  void readSensors()
  {
    if (readPMSdata(&pmsSerial)) {
      // reading data was successful!
      Serial.println();
      Serial.println("Lectura de sensor PMS exitosa");
    }else{
      Serial.println();
      Serial.println("Falló lectura de sensor PMS");
    }
    delay(2000); //Es un sensor lento, por lo que hay que darle tiempo.
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    //CALCULO SENSORES MQ
    int adc_MQ7 = analogRead(ad_mq7); //Leemos la salida analógica del MQ
    float voltaje_mq7 = adc_MQ7 * (v_ref / 4095.0); //Convertimos la lectura en un valor de voltaje
    int adc_MQ135 = analogRead(ad_mq135); //Leemos la salida analógica del MQ
    float voltaje_mq135 = adc_MQ135 * (v_ref  / 4095.0); //Convertimos la lectura en un valor de voltaje
    
    float Rs=1000/3*((5-voltaje_mq135)/voltaje_mq135);  //Calculamos Rs con un RL de 1k
    co2_ppm =factor*pow(Rs/Ro, exponente);
  
    float Rs_CO = 1000/3*((5-voltaje_mq7)/voltaje_mq7);  //Calculamos Rs con un RL de 1k
    co_ppm = factor_co*pow(Rs_CO/Ro_co, exponente_co); // calculamos la concentración  de alcohol con la ecuación obtenida.

    if (isnan(h) || isnan(t)) {
      Serial.println(F("Fallo en la lectura del sensor DHT!"));
      //h = 61;
      //t = 28;
      return;
    }
    //Connect Sensor on Analogic PIN A0
    int sensor1 = analogRead(0);
    measures[0][0] = "temperature";
    measures[0][1] = String(t);
    //Connect Sensor on Analogic PIN A1
    int sensor2 = analogRead(1);
    measures[1][0] = "relativeHumidity";
    measures[1][1] = String(h);
  }

  boolean readPMSdata(Stream *s) {
    if (! s->available()) {
      return false;
    }
    // Read a byte at a time until we get to the special '0x42' start-byte
    if (s->peek() != 0x42) {
      s->read();
      return false;
    }
    // Now read all 32 bytes
    if (s->available() < 32) {
      return false;
    } 
    uint8_t buffer[32];    
    uint16_t sum = 0;
    s->readBytes(buffer, 32);
    // get checksum ready
    for (uint8_t i=0; i<30; i++) {
      sum += buffer[i];
    }
    // The data comes in endian'd, this solves it so it works on all platforms
    uint16_t buffer_u16[15];
    for (uint8_t i=0; i<15; i++) {
      buffer_u16[i] = buffer[2 + i*2 + 1];
      buffer_u16[i] += (buffer[2 + i*2] << 8);
    }
    memcpy((void *)&data, (void *)buffer_u16, 30);
    if (sum != data.checksum) {
      Serial.println("Checksum failure");
      return false;
    }
    return true; // success!
}
