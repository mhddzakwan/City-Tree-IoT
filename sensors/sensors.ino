#include <Arduino.h>

// ===== DHT11 =====
#include "DHT.h"
#define DHTPIN 19
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ===== MH-Z19 =====
#include "MHZ19.h"
#define MHZ19_RX 4
#define MHZ19_TX 5
#define MHZ19_BAUD 9600
HardwareSerial mhzSerial(1);
MHZ19 mhz19;

// ===== SDS011 =====
#include "SdsDustSensor.h"
#define SDS_RX 16
#define SDS_TX 17
HardwareSerial sdsSerial(2);
SdsDustSensor sds(sdsSerial);

// ===== MQ-2 =====
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
const float RL = 5.0;
float R0 = 10.0;

// ===== LCD I2C =====
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ===== Variabel display =====
int displayState = 0;
unsigned long lastSwitch = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ===== DHT =====
  dht.begin();
  Serial.println("DHT11 siap di GPIO19");

  // ===== MH-Z19 =====
  Serial.println("Inisialisasi MH-Z19...");
  mhzSerial.begin(MHZ19_BAUD, SERIAL_8N1, MHZ19_RX, MHZ19_TX);
  mhz19.begin(mhzSerial);
  mhz19.autoCalibration(true); // Auto calibration ON
  Serial.println("MH-Z19 Siap.");

  // ===== SDS011 =====
  Serial.println("Inisialisasi SDS011...");
  sdsSerial.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  delay(800);
  sds.begin();
  Serial.println("SDS011 Siap.");

  // ===== MQ-2 =====
  if (!ads.begin()) {
    Serial.println("Gagal deteksi ADS1115!");
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  Serial.println("ADS1115 (MQ2) Siap.");

  // ===== LCD =====
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Monitoring Air Quality");
  lcd.setCursor(0, 1); lcd.print("ESP32 + LCD I2C Ready");
  lcd.setCursor(0, 2); lcd.print("Sensors Initializing...");
  delay(1500);
}

void loop() {
  // Baca DHT11
  float hum = dht.readHumidity();
  float tempDHT = dht.readTemperature();

  // Baca MH-Z19
  int co2 = mhz19.getCO2();
  int8_t tempCO2 = mhz19.getTemperature();

  // Baca SDS011
  PmResult pm = sds.readPm();

  // Baca MQ-2
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.1875 / 1000;
  float RS = ((5.0 * RL) / voltage) - RL;
  float ratio = RS / R0;
  float ppm = pow(10, ((log10(ratio) - 0.48) / -0.38));
  if (ppm > 1000) ppm = 1000;

  // ===== DISPLAY LCD BERGANTIAN =====
  if (millis() - lastSwitch >= 2000) {
    displayState++;
    if (displayState > 3) displayState = 0;
    lastSwitch = millis();
  }

  lcd.clear();
  switch (displayState) {
    case 0:
      lcd.setCursor(0,0); lcd.print("DHT11 Temp/Hum:");
      lcd.setCursor(0,1); lcd.print("Suhu : "); lcd.print(tempDHT); lcd.print(" C");
      lcd.setCursor(0,2); lcd.print("Humid: "); lcd.print(hum); lcd.print(" %");
      break;

    case 1:
      lcd.setCursor(0,0); lcd.print("MH-Z19 CO2 & Temp:");
      lcd.setCursor(0,1); lcd.print("CO2 : "); lcd.print(co2); lcd.print(" ppm");
      lcd.setCursor(0,2); lcd.print("Temp: "); lcd.print(tempCO2); lcd.print(" C");
      break;

    case 2:
      lcd.setCursor(0,0); lcd.print("SDS011 PM2.5/PM10");
      lcd.setCursor(0,1); lcd.print("PM2.5: "); lcd.print(pm.pm25);
      lcd.setCursor(0,2); lcd.print("PM10 : "); lcd.print(pm.pm10);
      break;

    case 3:
      lcd.setCursor(0,0); lcd.print("MQ-2 Gas Smoke CO:");
      lcd.setCursor(0,1); lcd.print("CO : ");
      lcd.print(ppm, 1); lcd.print(" ppm");
      lcd.setCursor(0,2); lcd.print("RS/R0: "); lcd.print(ratio, 2);
      break;
  }

  delay(200);
}
