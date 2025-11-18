#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 9);  // RX, TX
                               // Pin 8 = RX, Pin 9 = TX

String dataMasuk = "";

void setup() {
  Serial.begin(9600);          // Serial monitor
  mySerial.begin(9600);        // Software serial UART ke ESP32

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  Serial.println("Arduino siap menerima perintah...");
}

void loop() {
  if (mySerial.available()) {
    dataMasuk = mySerial.readStringUntil('\n');
    dataMasuk.trim();

    if (dataMasuk.equalsIgnoreCase("ON")) {
      digitalWrite(2, HIGH);
      Serial.println("Lampu ON");
    }
    else if (dataMasuk.equalsIgnoreCase("OFF")) {
      digitalWrite(2, LOW);
      Serial.println("Lampu OFF");
    }
    else {
      Serial.println("Perintah tidak dikenali!");
    }
  }
}
