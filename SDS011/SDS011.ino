#include <HardwareSerial.h>
#include "SdsDustSensor.h"

int rxPin = 18;  // SDS011 TX -> ESP32 RX
int txPin = 19;  // SDS011 RX <- ESP32 TX

HardwareSerial& sdsSerial = Serial2;
SdsDustSensor sds(sdsSerial);

void setup() {
  Serial.begin(115200);
  sdsSerial.begin(9600, SERIAL_8N1, rxPin, txPin);
  delay(1000);

  Serial.println("Inisialisasi SDS011...");
  sds.begin();
  delay(1000); // beri waktu sensor untuk startup

  Serial.println(sds.setActiveReportingMode().toString());  // Mode aktif (otomatis kirim data)
  Serial.println(sds.setCustomWorkingPeriod(0).toString()); // Mode continuous (selalu aktif)
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
    Serial.println("Gagal membaca data dari SDS011");
  }

  delay(2000); // baca setiap 2 detik
}
