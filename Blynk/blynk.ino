/****************************************************
  ESP32 Air Quality Monitor + Blynk + FreeRTOS
****************************************************/

#define BLYNK_TEMPLATE_ID "TMPL6NXx-CXOj"
#define BLYNK_TEMPLATE_NAME "Breathfora"
#define BLYNK_AUTH_TOKEN "lFayDapYeH2bSla-T4IsSbOJ2_D8i1-e"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// ====== WiFi ======
char ssid[] = "KelilingDunia1";
char pass[] = "12julham12";

// ====== Preferences ======
#include <Preferences.h>
Preferences prefs;

// ====== DHT ======
#include "DHT.h"
#define DHTPIN 19
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ====== MH-Z19 ======
#include "MHZ19.h"
#define MHZ19_RX 4
#define MHZ19_TX 5
HardwareSerial mhzSerial(1);
MHZ19 mhz19;

// ====== SDS011 ======
#include "SdsDustSensor.h"
#define SDS_RX 16
#define SDS_TX 17
HardwareSerial sdsSerial(2);
SdsDustSensor sds(sdsSerial);

// ====== ADS1115 (MQ2) ======
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
const float RL = 5.0;
float R0 = 10.0;

// ====== LCD ======
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ====== Relay Pins ======
#define RELAY_HIJAU   13
#define RELAY_KUNING  12
#define RELAY_MERAH   14
#define RELAY_KIPAS   27
#define RELAY_IONIZER 33
#define RELAY_MIST    32

// ====== Mutex ======
SemaphoreHandle_t printMutex;
SemaphoreHandle_t i2cMutex;

// ====== Shared Variables ======
int lastHum = NAN;
int lastTempDHT = NAN;
int lastCO2 = -1;
int lastPM25 = -1;
int lastPM10 = -1;
int lastMQ2ppm = -1;

int threshTemp = 0;
int threshHum = 0;
int threshCO2 = 0;
int threshPM25 = 0;
int threshPM10 = 0;
int threshCO = 0;

int kipas = 0;
int danger = 0;

// ====== EEPROM (Preferences) ======
void loadThreshold() {
  prefs.begin("thresh", true);  // read-only
  threshTemp = prefs.getInt("tTemp", 30);
  threshHum  = prefs.getInt("tHum", 70);
  threshCO2  = prefs.getInt("tCO2", 800);
  threshPM25 = prefs.getInt("tPM25", 35);
  threshPM10 = prefs.getInt("tPM10", 50);
  threshCO   = prefs.getInt("tCO", 200);
  prefs.end();
}

void saveThreshold() {
  prefs.begin("thresh", false); // write mode
  prefs.putInt("tTemp", threshTemp);
  prefs.putInt("tHum",  threshHum);
  prefs.putInt("tCO2",    threshCO2);
  prefs.putInt("tPM25", threshPM25);
  prefs.putInt("tPM10", threshPM10);
  prefs.putInt("tCO",   threshCO);
  prefs.end();
}



// ===== CEK DANGER =====
void cekDanger() {
  danger = 0;

  if (lastTempDHT > threshTemp) danger++;
  if (lastHum > threshHum) danger++;
  if (lastCO2 > threshCO2) danger++;
  if (lastPM25 > threshPM25) danger++;
  if (lastPM10 > threshPM10) danger++;
  if (lastMQ2ppm > threshCO) danger++;

  // ======== KONTROL LAMPU ========
  if (danger == 0) {
    digitalWrite(RELAY_HIJAU, LOW);
    digitalWrite(RELAY_KUNING, HIGH);
    digitalWrite(RELAY_MERAH, HIGH);

    digitalWrite(RELAY_IONIZER, HIGH);
    digitalWrite(RELAY_MIST, HIGH);
  }
  else if (danger <= 3) {
    digitalWrite(RELAY_HIJAU, HIGH);
    digitalWrite(RELAY_KUNING, LOW);
    digitalWrite(RELAY_MERAH, HIGH);

    digitalWrite(RELAY_IONIZER, LOW);
    digitalWrite(RELAY_MIST, LOW);
  }
  else {
    digitalWrite(RELAY_HIJAU, HIGH);
    digitalWrite(RELAY_KUNING, HIGH);
    digitalWrite(RELAY_MERAH, LOW);

    digitalWrite(RELAY_IONIZER, LOW);
    digitalWrite(RELAY_MIST, LOW);
  }

  // ======== KIPAS DARI BLYNK ========
  digitalWrite(RELAY_KIPAS, kipas ? LOW : HIGH);
}


// ===== SENSOR TASK =====
void TaskSensor(void *pv) {
  const TickType_t xDelay = pdMS_TO_TICKS(2000);

  for (;;) {
    float hum = dht.readHumidity();
    float tempDHT = dht.readTemperature();
    int co2 = mhz19.getCO2();
    PmResult pm = sds.readPm();

    float ppm = NAN;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      int16_t adc0 = ads.readADC_SingleEnded(0);
      float voltage = adc0 * 0.1875 / 1000.0;

      if (voltage > 0.001) {
        float RS = ((5.0 * RL) / voltage) - RL;
        float ratio = RS / R0;
        if (ratio > 0) {
          ppm = pow(10.0, ((log10(ratio) - 0.48) / -0.38));
          if (ppm > 1000) ppm = 1000;
        }
      }

      lastHum = hum;
      lastTempDHT = tempDHT;
      lastCO2 = co2;

      if (pm.isOk()) {
        lastPM25 = pm.pm25;
        lastPM10 = pm.pm10;
      }

      lastMQ2ppm = ppm;

      // ====== LCD ======
      lcd.clear();

      // Baris 0 → Suhu & Humidity
      lcd.setCursor(0, 0);
      lcd.print("T:"); lcd.print((int)tempDHT);
      lcd.print("("); lcd.print(getStatus(tempDHT, threshTemp)); lcd.print(") ");

      lcd.print("H:"); lcd.print((int)hum);
      lcd.print("("); lcd.print(getStatus(hum, threshHum)); lcd.print(")");

      // Baris 1 → CO2
      lcd.setCursor(0, 1);
      lcd.print("C02:"); lcd.print(co2); lcd.print("ppm ");
      lcd.print("("); lcd.print(getStatus(co2, threshCO2)); lcd.print(")");

      // Baris 2 → PM2.5 & PM10
      lcd.setCursor(0, 2);
      lcd.print("P2.5:"); lcd.print(lastPM25);
      lcd.print("("); lcd.print(getStatus(lastPM25, threshPM25)); lcd.print(") ");

      lcd.print("P10:"); lcd.print(lastPM10);
      lcd.print("("); lcd.print(getStatus(lastPM10, threshPM10)); lcd.print(")");

      // Baris 3 → MQ2
      lcd.setCursor(0, 3);
      lcd.print("CO:");
      if (!isnan(lastMQ2ppm)) {
          lcd.print(lastMQ2ppm);
          lcd.print("(");
          lcd.print(getStatus(lastMQ2ppm, threshCO));
          lcd.print(")");
      } else {
          lcd.print("NA");
      }

      xSemaphoreGive(i2cMutex);

    }

    // ===== SEND TO BLYNK =====
    Blynk.virtualWrite(V0, lastTempDHT);
    Blynk.virtualWrite(V1, lastHum);
    Blynk.virtualWrite(V2, lastCO2);
    Blynk.virtualWrite(V3, lastPM25);
    Blynk.virtualWrite(V4, lastPM10);
    Blynk.virtualWrite(V5, lastMQ2ppm);

    cekDanger();  // <— JALANKAN SISTEM RELAY

    vTaskDelay(xDelay);
  }
}


// ===== BLYNK CALLBACK =====

BLYNK_WRITE(V6) {
  threshTemp = param.asFloat();
  saveThreshold();
}
BLYNK_WRITE(V7) {
  threshHum = param.asFloat();
  saveThreshold();
}
BLYNK_WRITE(V8) {
  threshCO2 = param.asInt();
  saveThreshold();
}
BLYNK_WRITE(V9) {
  threshPM25 = param.asFloat();
  saveThreshold();
}
BLYNK_WRITE(V10) {
  threshPM10 = param.asFloat();
  saveThreshold();
}
BLYNK_WRITE(V11) {
  threshCO = param.asFloat();
  saveThreshold();
}

BLYNK_WRITE(V12) {    // ★★ KIPAS ★★
  kipas = param.asInt();
}

String getStatus(float value, float thresh) {
  if (value <= thresh) return "OK";
  else return "NG";
}


// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_HIJAU, OUTPUT);
  pinMode(RELAY_KUNING, OUTPUT);
  pinMode(RELAY_MERAH, OUTPUT);
  pinMode(RELAY_KIPAS, OUTPUT);
  pinMode(RELAY_IONIZER, OUTPUT);
  pinMode(RELAY_MIST, OUTPUT);

  digitalWrite(RELAY_HIJAU, HIGH);
  digitalWrite(RELAY_KUNING, HIGH);
  digitalWrite(RELAY_MERAH, HIGH);
  digitalWrite(RELAY_KIPAS, HIGH);
  digitalWrite(RELAY_IONIZER, HIGH);
  digitalWrite(RELAY_MIST, HIGH);

  printMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin();

  lcd.begin();
  lcd.backlight();

  dht.begin();

  mhzSerial.begin(9600, SERIAL_8N1, MHZ19_RX, MHZ19_TX);
  mhz19.begin(mhzSerial);
  lcd.print("Menyiapkan Sensor...2");
  delay(3000);
  lcd.clear();
  

  sdsSerial.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  sds.begin();

  ads.begin();
  ads.setGain(GAIN_TWOTHIRDS);
  lcd.print("Menyiapkan Sensor...1"); 
  delay(3000);
  
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  loadThreshold();

  xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 1, NULL, 1);
}

void loop() {
  Blynk.run();
}
