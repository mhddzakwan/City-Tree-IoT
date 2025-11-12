#include <Arduino.h>
#include "MHZ19.h"
#include "SdsDustSensor.h"

// ====== MH-Z19 (CO2 Sensor) ======
#define MHZ19_RX 4   // Pin RX untuk MH-Z19 (ke TX sensor)
#define MHZ19_TX 5   // Pin TX untuk MH-Z19 (ke RX sensor)
#define MHZ19_BAUD 9600
HardwareSerial mhzSerial(1); // Gunakan UART1
MHZ19 mhz19;

// ====== SDS011 (Dust Sensor) ======
#define SDS_RX 16  // SDS011 TX -> ESP32 RX
#define SDS_TX 17  // SDS011 RX <- ESP32 TX
HardwareSerial sdsSerial(2); // Gunakan UART2
SdsDustSensor sds(sdsSerial);

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inisialisasi MH-Z19
  Serial.println("Inisialisasi MH-Z19...");
  mhzSerial.begin(MHZ19_BAUD, SERIAL_8N1, MHZ19_RX, MHZ19_TX);
  mhz19.begin(mhzSerial);
  mhz19.autoCalibration(false);
  Serial.println("MH-Z19 Siap.");

  // Inisialisasi SDS011
  Serial.println("Inisialisasi SDS011...");
  sdsSerial.begin(9600, SERIAL_8N1, SDS_RX, SDS_TX);
  delay(1000);
  sds.begin();
  delay(1000);
  Serial.println(sds.setActiveReportingMode().toString());
  Serial.println(sds.setCustomWorkingPeriod(0).toString());
  Serial.println("SDS011 Siap.");

  Serial.println("=== Sistem Deteksi Udara Siap ===");
}

// ====== Loop ======
void loop() {
  // ==== Baca MH-Z19 ====
  int co2 = mhz19.getCO2();
  int8_t temp = mhz19.getTemperature();

  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.print(" ppm | Suhu: ");
  Serial.print(temp);
  Serial.println(" °C");

  // ==== Baca SDS011 ====
  PmResult pm = sds.readPm();
  if (pm.isOk()) {
    Serial.print("PM2.5: ");
    Serial.print(pm.pm25);
    Serial.print(" µg/m³, PM10: ");
    Serial.print(pm.pm10);
    Serial.println(" µg/m³");
  } else {
    Serial.println("Gagal membaca SDS011");
  }

  Serial.println("-----------------------------");
  delay(5000); // baca setiap 5 detik
}
