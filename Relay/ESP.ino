#define RELAY1 12
#define RELAY2 13
#define RELAY3 14
#define RELAY4 27
#define RELAY5 32
#define RELAY6 33

void setup() {
  Serial.begin(115200);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);

  // Semua relay OFF (aktif LOW)
  relayOff(1);
  relayOff(2);
  relayOff(3);
  relayOff(4);
  relayOff(5);
  relayOff(6);

  Serial.println("Ketik contoh: 1 1 atau 3 0");
}

void loop() {
  if (Serial.available() > 0) {
    int relayNum = Serial.parseInt();  // baca nomor relay
    int state = Serial.parseInt();     // baca status 0/1

    if (relayNum >= 1 && relayNum <= 6 && (state == 0 || state == 1)) {
      setRelay(relayNum, state);
      Serial.print("Relay ");
      Serial.print(relayNum);
      Serial.print(" -> ");
      Serial.println(state == 1 ? "ON" : "OFF");
    }

    // bersihkan buffer
    while (Serial.available()) Serial.read();
  }
}

void setRelay(int num, int state) {
  int pin;
  switch (num) {
    case 1: pin = RELAY1; break;
    case 2: pin = RELAY2; break;
    case 3: pin = RELAY3; break;
    case 4: pin = RELAY4; break;
    case 5: pin = RELAY5; break;
    case 6: pin = RELAY6; break;
  }

  digitalWrite(pin, state == 1 ? LOW : HIGH);  // Aktif LOW
}

void relayOff(int num) { setRelay(num, 0); }
