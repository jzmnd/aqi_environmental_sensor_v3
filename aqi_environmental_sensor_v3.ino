/*
   AQI Environmental Sensor v3.0
    Jez Smith
    Using PMS5003 particle sensor and HIH6130 humidity/temperature sensor

    For use with Arduino MKR 1010 (SAMD)
    With MQTT smart home integration
*/
#define DEBUG_SER

#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "wiring_private.h"
#include <HIH61xx.h>
#include <AsyncDelay.h>
#include <LiquidCrystal_I2C.h>
#include <PMS5003Sensor.h>

// Wifi and MQTT
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "10.0.0.94";
const int mqttPort = 1883;
const char mqttUsername[] = SECRET_MQTT_USER;
const char mqttPassword[] = SECRET_MQTT_PASS;
const char topicPm1_0[] = "home/familyroom/aqi_pms_pm_1_0_topic";
const char topicPm2_5[] = "home/familyroom/aqi_pms_pm_2_5_topic";
const char topicPm10_0[] = "home/familyroom/aqi_pms_pm_10_0_topic";
const char topicAmbientTemp[] = "home/familyroom/aqi_hih_ambient_temp_topic";
const char topicRelHumidity[] = "home/familyroom/aqi_hih_rel_humidity_topic";

// Serial connection for PMS sensor
const int PMS_RX_PIN = 1;
const int PMS_TX_PIN = 0;
Uart pmsSerial(&sercom3, PMS_RX_PIN, PMS_TX_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);
PMS5003Sensor<Uart> pms(pmsSerial);

// LCD
LiquidCrystal_I2C lcd(0x26, 16, 2);
unsigned long currentMillis;
unsigned long previousMillis = 0;
const unsigned long refreshInterval = 4000;  // refresh the LCD every 4 sec (4000 ms) without blocking
bool pmsView = true;

// Logging
const char logFileName[] = "datalog.csv";

// The "hih" object must be created with a reference to the "Wire" object which represents the I2C bus it is using.
// Note that the class for the Wire object is called "TwoWire", and must be included in the templated class name.
HIH61xx<TwoWire> hih(Wire, 0x27);
AsyncDelay samplingIntervalHIH;
const unsigned long sampleInterval = 1000;  // HIH sample interval of 1 sec (1000 ms)
bool printed = true;

// Function to blink LED n times
void blink(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

// Attach the interrupt handler to the SERCOM
void SERCOM3_Handler() {
  pmsSerial.IrqHandler();
}

// Function to connect to the WiFi
void wifiConnect() {
  int wifiStatus = WL_IDLE_STATUS;
  while (wifiStatus != WL_CONNECTED) {
#ifdef DEBUG_SER
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
#endif
    // Connect to WPA/WPA2 network
    wifiStatus = WiFi.begin(ssid, pass);
    // Wait 10 seconds for connection
    delay(10000);
  }
}

// Function to test WiFi connection status and reconnect
void testWifiConnection() {
  int wifiStatus = WiFi.status();
  if (wifiStatus == WL_CONNECTION_LOST || wifiStatus == WL_DISCONNECTED || wifiStatus == WL_SCAN_COMPLETED) {
    wifiConnect();
  }
}

// Function to connect to MQTT client
void mqttConnect() {
  mqttClient.setUsernamePassword(mqttUsername, mqttPassword);
#ifdef DEBUG_SER
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(mqttBroker);
#endif
  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (true);
  }
}

// Function to test MQTT connection and reconnect
void testMqttConnection() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT connection lost");
    mqttConnect();
  }
}

// Function to write log to SD card
void logSD() {
  File dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(String(WiFi.getTime()));
    dataFile.print(",");
    dataFile.print(WiFi.localIP());
    dataFile.print(",");
    dataFile.print(String(hih.getAmbientTemp()));
    dataFile.print(",");
    dataFile.print(String(hih.getRelHumidity()));
    dataFile.print(",");
    dataFile.print(String(hih.getStatus()));
    dataFile.print(",");
    dataFile.print(pms.pm1_0str());
    dataFile.print(",");
    dataFile.print(pms.pm2_5str());
    dataFile.print(",");
    dataFile.print(pms.pm10_0str());
    dataFile.print(",");
    dataFile.print(pms.pm1_0envstr());
    dataFile.print(",");
    dataFile.print(pms.pm2_5envstr());
    dataFile.print(",");
    dataFile.print(pms.pm10_0envstr());
    dataFile.print(",");
    dataFile.print(pms.particles_0_3um_str());
    dataFile.print(",");
    dataFile.print(pms.particles_0_5um_str());
    dataFile.print(",");
    dataFile.print(pms.particles_1_0um_str());
    dataFile.print(",");
    dataFile.print(pms.particles_2_5um_str());
    dataFile.print(",");
    dataFile.print(pms.particles_5_0um_str());
    dataFile.print(",");
    dataFile.println(pms.particles_10_0um_str());
    dataFile.close();
  } else {
    Serial.print("Error opening: ");
    Serial.println(logFileName);
  }
}

/*
   Setup function
    Sets up serial connections
    Sets up I2C connection for HIH6130
    Switches on LCD display
    Sets up SD card
    Connects to WiFi
    Connects to MQTT broker
*/
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // blink LED 4 times before initialization
  blink(4);
  // debug output
  Serial.begin(115200);
#ifdef DEBUG_SER
  while (!Serial);
#endif
  // PMS sensor output
  pmsSerial.begin(9600);
  pinPeripheral(PMS_RX_PIN, PIO_SERCOM);  // Assign RX function to pin 1
  pinPeripheral(PMS_TX_PIN, PIO_SERCOM);  // Assign TX function to pin 0
  // I2C setup for HIH sensor
  Wire.begin();
  hih.initialise();
  samplingIntervalHIH.start(sampleInterval, AsyncDelay::MILLIS);
  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("AQI Env Sensor");
  lcd.setCursor(0, 1);
  lcd.print("Jez Smith");
  blink(2);
  delay(1000);
  // SD card setup
  if (!SD.begin(SD_CHIP_SELECT_PIN)) {
#ifdef DEBUG_SER
    Serial.println("SD initialization failed!");
#endif
    while (true);
  }
  // Attempt to connect to Wifi network
  wifiConnect();
  // Attempt to connect to MQTT broker
  mqttConnect();

#ifdef DEBUG_SER
  Serial.println("Initialization complete.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
#endif
}

/*
   Main loop
     Read the PMS data
     Read the HIH data
     Log data and send to MQTT
     Update the LCD
*/
void loop() {
  // Call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  // Check Wifi and MQTT connections
  testWifiConnection();
  testMqttConnection();

  // Read PMS sensor data
  if (pms.read()) {
#ifdef DEBUG_SER
    Serial.println("-------------");
    pms.print_cu_std();
    Serial.println("-------------");
    pms.print_cu_env();
    Serial.println("-------------");
    pms.print_particles();
    Serial.println("-------------");
#endif
  }

  // Read HIH sensor data
  if (samplingIntervalHIH.isExpired() && !hih.isSampling()) {
    hih.start();
    printed = false;
    samplingIntervalHIH.repeat();
  }
  hih.process();
  if (hih.isFinished() && !printed) {
    printed = true;
#ifdef DEBUG_SER
    Serial.print("RH: ");
    Serial.print(hih.getRelHumidity() / 100.0);
    Serial.println(" %");
    Serial.print("Ambient: ");
    Serial.print(hih.getAmbientTemp() / 100.0);
    Serial.println(" deg C");
    Serial.print("Status: ");
    Serial.println(hih.getStatus());
    Serial.println("-------------");
#endif
  }

  // Update LCD and write out to SD
  currentMillis = millis();
  if (currentMillis - previousMillis > refreshInterval) {
    previousMillis = currentMillis;
    // Log data to SD card
    logSD();
    // Send data to MQTT broker
    mqttClient.beginMessage(topicPm1_0);
    mqttClient.print(pms.pm1_0str().toFloat());
    mqttClient.endMessage();
    mqttClient.beginMessage(topicPm2_5);
    mqttClient.print(pms.pm2_5str().toFloat());
    mqttClient.endMessage();
    mqttClient.beginMessage(topicPm10_0);
    mqttClient.print(pms.pm10_0str().toFloat());
    mqttClient.endMessage();
    mqttClient.beginMessage(topicAmbientTemp);
    mqttClient.print(hih.getAmbientTemp() / 100.0);
    mqttClient.endMessage();
    mqttClient.beginMessage(topicRelHumidity);
    mqttClient.print(hih.getRelHumidity() / 100.0);
    mqttClient.endMessage();
    // Alternate LCD between PMS data and HIH data
    if (pmsView) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PM1  PM2.5  PM10");
      lcd.setCursor(0, 1);
      lcd.print(pms.pm1_0str());
      lcd.setCursor(5, 1);
      lcd.print(pms.pm2_5str());
      lcd.setCursor(12, 1);
      lcd.print(pms.pm10_0str());
      pmsView = false;
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TMP C   RH %");
      lcd.setCursor(0, 1);
      lcd.print(hih.getAmbientTemp() / 100.0, 1);
      lcd.setCursor(8, 1);
      lcd.print(hih.getRelHumidity() / 100.0, 1);
      pmsView = true;
    }
  }
}
