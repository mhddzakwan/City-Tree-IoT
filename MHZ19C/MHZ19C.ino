#include <Arduino.h>
#include "MHZ19.h"

#define MHZ19_RX 4   // Pin RX untuk MH-Z19 (ke TX sensor)
#define MHZ19_TX 5   // Pin TX untuk MH-Z19 (ke RX sensor)
#define MHZ19_BAUD 9600

HardwareSerial mhzSerial(1); // Gunakan UART1
MHZ19 mhz19;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inisialisasi MH-Z19...");
  mhzSerial.begin(MHZ19_BAUD, SERIAL_8N1, MHZ19_RX, MHZ19_TX);
  mhz19.begin(mhzSerial);
  mhz19.autoCalibration(false);
  Serial.println("MH-Z19 Siap.");
}

void loop() {
  int co2 = mhz19.getCO2();
  int8_t temp = mhz19.getTemperature();

  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.print(" ppm | Suhu: ");
  Serial.print(temp);
  Serial.println(" °C");

  Serial.println("-----------------------------");
  delay(5000);
}
