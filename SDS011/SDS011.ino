#include <Arduino.h>
#include "SdsDustSensor.h"

#define SDS_RX 16  // SDS011 TX -> ESP32 RX
#define SDS_TX 17  // SDS011 RX <- ESP32 TX
#define SDS_BAUD 9600

HardwareSerial sdsSerial(2); // Gunakan UART2
SdsDustSensor sds(sdsSerial);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inisialisasi SDS011...");
  sdsSerial.begin(SDS_BAUD, SERIAL_8N1, SDS_RX, SDS_TX);
  delay(1000);

  sds.begin();
  delay(1000);
  Serial.println(sds.setActiveReportingMode().toString());
  Serial.println(sds.setCustomWorkingPeriod(0).toString());
  Serial.println("SDS011 Siap.");
}

void loop() {
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
  delay(5000);
}
