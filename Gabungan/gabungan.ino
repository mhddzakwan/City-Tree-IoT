/*
  Gabungan Sensor + Relay dengan FreeRTOS pada ESP32
  - DHT11 (GPIO19)
  - MH-Z19 (UART1 RX=4 TX=5)
  - SDS011 (UART2 RX=16 TX=17)
  - MQ-2 via ADS1115 (I2C)
  - LCD I2C 0x27 (sama bus I2C dengan ADS1115)
  - 6x Relay (aktif LOW): pins 12,13,14,27,32,33
  Serial commands (kirim lewat Serial Monitor):
    "<relay> <0|1>"   e.g. "1 1" -> Relay1 ON
    "all 0" / "all 1" -> semua relay OFF/ON
    "status"          -> tampilkan status semua relay & sensor singkat
*/

#include <Arduino.h>

// ====== RELAY PINS ======
#define RELAY1 12
#define RELAY2 13
#define RELAY3 14
#define RELAY4 27
#define RELAY5 32
#define RELAY6 33

// ====== DHT11 ======
#include "DHT.h"
#define DHTPIN 19
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ====== MH-Z19 ======
#include "MHZ19.h"
#define MHZ19_RX 4
#define MHZ19_TX 5
#define MHZ19_BAUD 9600
HardwareSerial mhzSerial(1);
MHZ19 mhz19;

// ====== SDS011 ======
#include "SdsDustSensor.h"
#define SDS_RX 16
#define SDS_TX 17
HardwareSerial sdsSerial(2);
SdsDustSensor sds(sdsSerial);

// ====== MQ-2 via ADS1115 (I2C) ======
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
const float RL = 5.0;    // kilo ohm
float R0 = 10.0;         // harus dikalibrasi

// ====== LCD I2C ======
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ====== FreeRTOS sync ======
SemaphoreHandle_t printMutex;   // untuk melindungi Serial printing
SemaphoreHandle_t i2cMutex;     // untuk melindungi operasi I2C/LCD/ADS1115

// ====== Shared sensor variables (dilindungi oleh i2cMutex saat update) ======
float lastHum = NAN;
float lastTempDHT = NAN;
int lastCO2 = -1;
int lastTempCO2 = -128;
float lastPM25 = -1;
float lastPM10 = -1;
float lastMQ2ppm = -1;
float lastMQ2ratio = -1;

// ====== Relay states ======
bool relayState[7]; // index 1..6

// ====== Utility: set relay (active LOW) ======
void setRelay(int num, int state) {
  int pin = -1;
  switch (num) {
    case 1: pin = RELAY1; break;
    case 2: pin = RELAY2; break;
    case 3: pin = RELAY3; break;
    case 4: pin = RELAY4; break;
    case 5: pin = RELAY5; break;
    case 6: pin = RELAY6; break;
    default: return;
  }
  // state==1 -> ON -> output LOW (aktif LOW)
  digitalWrite(pin, state == 1 ? LOW : HIGH);
  relayState[num] = (state == 1);
}

// set all off (helper)
void relayAll(int state) {
  for (int i = 1; i <= 6; ++i) setRelay(i, state);
}

// ====== Task: Baca sensor & update LCD ======
void TaskSensor(void *pvParameters) {
  (void) pvParameters;
  const TickType_t xDelay = pdMS_TO_TICKS(2000); // 2s
  for (;;) {
    // --- BACA DHT ---
    float hum = dht.readHumidity();
    float tempDHT = dht.readTemperature();

    // --- BACA MH-Z19 ---
    int co2 = mhz19.getCO2();
    int8_t tempCO2 = mhz19.getTemperature();

    // --- BACA SDS011 ---
    PmResult pm = sds.readPm();

    // --- BACA MQ-2 via ADS1115 ---
    float voltage = 0.0;
    float RS = NAN;
    float ratio = NAN;
    float ppm = NAN;

    // akses I2C kritikal: gunakan i2cMutex
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
      // ADS1115 read
      int16_t adc0 = ads.readADC_SingleEnded(0);
      // ADS1115 default LSB 0.1875mV in GAIN_TWOTHIRDS
      voltage = adc0 * 0.1875 / 1000.0; // volt
      if (voltage > 0.001) {
        RS = ((5.0 * RL) / voltage) - RL;
        ratio = RS / R0;
        if (ratio > 0) {
          ppm = pow(10.0, ((log10(ratio) - 0.48) / -0.38));
          if (ppm > 1000) ppm = 1000;
        }
      }

      // Update shared sensor vars
      lastHum = hum;
      lastTempDHT = tempDHT;
      lastCO2 = co2;
      lastTempCO2 = tempCO2;
      if (pm.isOk()) {
        lastPM25 = pm.pm25;
        lastPM10 = pm.pm10;
      } else {
        // tidak diubah jika gagal baca
      }
      lastMQ2ppm = ppm;
      lastMQ2ratio = ratio;

      // Update LCD (pakai i2cMutex)
      lcd.clear();
      static uint8_t screen = 0;
      screen = (screen + 1) % 4;
      switch (screen) {
        case 0:
          lcd.setCursor(0,0); lcd.print("DHT11 Temp/Hum:");
          lcd.setCursor(0,1);
          if (!isnan(tempDHT)) { lcd.print("T:"); lcd.print(tempDHT,1); lcd.print(" C"); }
          else lcd.print("T: ERR");
          lcd.setCursor(0,2);
          if (!isnan(hum)) { lcd.print("H:"); lcd.print(hum,1); lcd.print(" %"); }
          else lcd.print("H: ERR");
          break;
        case 1:
          lcd.setCursor(0,0); lcd.print("MH-Z19 CO2:");
          lcd.setCursor(0,1);
          if (co2 >= 0) { lcd.print("CO2:"); lcd.print(co2); lcd.print(" ppm"); }
          else lcd.print("CO2: ERR");
          lcd.setCursor(0,2); lcd.print("T:"); lcd.print(tempCO2); lcd.print(" C");
          break;
        case 2:
          lcd.setCursor(0,0); lcd.print("SDS011 PM:");
          lcd.setCursor(0,1);
          if (pm.isOk()) { lcd.print("PM2.5:"); lcd.print(pm.pm25,1); }
          else lcd.print("PM2.5: ERR");
          lcd.setCursor(0,2);
          if (pm.isOk()) { lcd.print("PM10:"); lcd.print(pm.pm10,1); }
          break;
        case 3:
          lcd.setCursor(0,0); lcd.print("MQ-2 Gas (CO):");
          lcd.setCursor(0,1);
          if (!isnan(ppm)) { lcd.print(ppm,1); lcd.print(" ppm"); }
          else lcd.print("CO: ERR");
          lcd.setCursor(0,2);
          if (!isnan(ratio)) { lcd.print("Rsr/R0:"); lcd.print(ratio,2); }
          break;
      }

      xSemaphoreGive(i2cMutex);
    } // end if take i2cMutex

    // --- Print ringkasan ke Serial (dengan printMutex) ---
    if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
      Serial.println("=== Sensor Update ===");
      if (!isnan(lastTempDHT) && !isnan(lastHum)) {
        Serial.print("DHT11 -> Temp: "); Serial.print(lastTempDHT,1); Serial.print(" C, Hum: "); Serial.print(lastHum,1); Serial.println(" %");
      } else {
        Serial.println("DHT11 -> ERROR");
      }
      if (lastCO2 >= 0) {
        Serial.print("MH-Z19 -> CO2: "); Serial.print(lastCO2); Serial.print(" ppm, T: "); Serial.print(lastTempCO2); Serial.println(" C");
      } else {
        Serial.println("MH-Z19 -> ERROR");
      }
      if (lastPM25 >= 0) {
        Serial.print("SDS011 -> PM2.5: "); Serial.print(lastPM25,1); Serial.print(" ug/m3, PM10: "); Serial.println(lastPM10,1);
      } else {
        Serial.println("SDS011 -> ERROR");
      }
      if (!isnan(lastMQ2ppm)) {
        Serial.print("MQ-2 -> CO est: "); Serial.print(lastMQ2ppm,1); Serial.print(" ppm, Rr/R0: "); Serial.println(lastMQ2ratio,2);
      } else {
        Serial.println("MQ-2 -> ERROR");
      }
      Serial.println("----------------------");
      xSemaphoreGive(printMutex);
    }

    vTaskDelay(xDelay);
  }
}

// ====== Task: Monitor Serial untuk perintah relay ======
void TaskRelaySerial(void *pvParameters) {
  (void) pvParameters;
  String line;
  for (;;) {
    // baca line non-blocking
    if (Serial.available()) {
      // baca sampai newline atau timeout singkat
      line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      // ambil mutex untuk print saat membalas
      if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
        Serial.print("Command diterima: ");
        Serial.println(line);
        xSemaphoreGive(printMutex);
      }

      // parsing: support "n s" , "all s" , "status"
      if (line.equalsIgnoreCase("status")) {
        if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
          Serial.println("==== STATUS ====");
          for (int i = 1; i <= 6; ++i) {
            Serial.print("Relay "); Serial.print(i); Serial.print(": ");
            Serial.println(relayState[i] ? "ON" : "OFF");
          }
          // sensor short
          Serial.print("DHT T/H: ");
          if (!isnan(lastTempDHT)) { Serial.print(lastTempDHT,1); Serial.print(" C / "); Serial.print(lastHum,1); Serial.println(" %"); }
          else Serial.println("ERR");
          Serial.print("CO2: "); if (lastCO2>=0) Serial.println(lastCO2); else Serial.println("ERR");
          Serial.print("PM2.5: "); if (lastPM25>=0) Serial.println(lastPM25); else Serial.println("ERR");
          Serial.print("MQ-2 CO ppm: "); if (!isnan(lastMQ2ppm)) Serial.println(lastMQ2ppm); else Serial.println("ERR");
          Serial.println("=================");
          xSemaphoreGive(printMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      // cek "all <0|1>"
      if (line.startsWith("all ")) {
        int val = -1;
        String s = line.substring(4); s.trim();
        val = s.toInt();
        if (val == 0 || val == 1) {
          relayAll(val);
          if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
            Serial.print("Semua relay -> "); Serial.println(val==1 ? "ON" : "OFF");
            xSemaphoreGive(printMutex);
          }
        } else {
          if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) { Serial.println("Perintah all invalid. Gunakan: all 0 atau all 1"); xSemaphoreGive(printMutex); }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      // parsing "n s"
      // contoh: "1 1" atau "3 0"
      int spaceIndex = line.indexOf(' ');
      if (spaceIndex > 0) {
        String part1 = line.substring(0, spaceIndex);
        String part2 = line.substring(spaceIndex + 1);
        part1.trim(); part2.trim();
        int relayNum = part1.toInt();
        int state = part2.toInt();
        if (relayNum >= 1 && relayNum <= 6 && (state == 0 || state == 1)) {
          setRelay(relayNum, state);
          if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
            Serial.print("Relay "); Serial.print(relayNum); Serial.print(" -> "); Serial.println(state==1 ? "ON" : "OFF");
            xSemaphoreGive(printMutex);
          }
        } else {
          if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
            Serial.println("Perintah invalid. Format: '<relay> <0|1>' cont: '1 1' , 'all 0' , 'status'");
            xSemaphoreGive(printMutex);
          }
        }
      } else {
        if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
          Serial.println("Perintah tidak dikenali. Gunakan 'status', 'all 1', atau 'n s' (contoh: '2 1').");
          xSemaphoreGive(printMutex);
        }
      }
    } // end if Serial.available

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(500);

  // buat mutex
  printMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  if (printMutex == NULL || i2cMutex == NULL) {
    // gagal buat mutex -> hang
    while (1);
  }

  // ===== init relay pins =====
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);

  // Semua relay OFF (aktif LOW => HIGH)
  for (int i=1;i<=6;i++) relayState[i]=false;
  relayAll(0);

  // ===== init sensors & peripherals =====
  dht.begin();

  // MH-Z19
  mhzSerial.begin(MHZ19_BAUD, SERIAL_8N1, MHZ19_RX, MHZ19_TX);
  mhz19.begin(mhzSerial);
  mhz19.autoCalibration(false); // nonaktifkan auto-cal jika ingin stabil

  // SDS011
  sdsSerial.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  sds.begin();

  // I2C devices (ADS1115 and LCD) - protected by i2cMutex
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
    Wire.begin(); // default pins SDA=21 SCL=22
    if (!ads.begin()) {
      if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
        Serial.println("Gagal deteksi ADS1115! Periksa koneksi I2C.");
        xSemaphoreGive(printMutex);
      }
      // jangan hang, tetap lanjut (MQ-2 tidak akan berfungsi)
    } else {
      ads.setGain(GAIN_TWOTHIRDS);
    }

    lcd.begin();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Monitoring Air Quality");
    lcd.setCursor(0,1); lcd.print("ESP32 + LCD I2C Ready");
    lcd.setCursor(0,2); lcd.print("Sensors Initializing...");
    delay(1000);
    xSemaphoreGive(i2cMutex);
  }

  // tampilan awal via Serial
  if (xSemaphoreTake(printMutex, pdMS_TO_TICKS(200))) {
    Serial.println("=== Sistem Inisialisasi Selesai ===");
    Serial.println("Kirim perintah: '1 1' atau '3 0' , 'all 1' , 'status'");
    Serial.println("Relay aktif = LOW");
    xSemaphoreGive(printMutex);
  }

  // Buat task FreeRTOS
  xTaskCreatePinnedToCore(TaskSensor, "TaskSensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskRelaySerial, "TaskRelay", 4096, NULL, 1, NULL, 1);

  // tidak perlu loop() heavy
}

// ====== loop kosong, kerja dilakukan di tasks ======
void loop() {
  // kosong - semua pekerjaan dilakukan di tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
