// ESP32: Mengirim perintah ON/OFF melalui UART ke Arduino

#define TX2 32   // TX2 ESP32
#define RX2 33   // RX2 ESP32

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);

  Serial.println("Ketik ON atau OFF untuk dikirim ke Arduino");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    Serial2.println(cmd);  // kirim ke Arduino
    Serial.println("Dikirim ke Arduino: " + cmd);
  }
}
