#include <Arduino.h>
#include "MHZ19.h"

#define RX_PIN 16
#define TX_PIN 17
#define BAUDRATE 9600

HardwareSerial mySerial(2);
MHZ19 myMHZ19;

void setup() {
  Serial.begin(9600);
  mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  myMHZ19.begin(mySerial);

  myMHZ19.autoCalibration(false); // nonaktifkan ABC sementara
}

void loop() {
  int co2 = myMHZ19.getCO2();
  int8_t temp = myMHZ19.getTemperature();

  Serial.print("CO2 (ppm): ");
  Serial.println(co2);
  Serial.print("Temperature (C): ");
  Serial.println(temp);

  delay(3000);
}
