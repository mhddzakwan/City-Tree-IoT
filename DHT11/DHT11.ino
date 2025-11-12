#include "DHT.h"

#define DHTPIN 19       // Pin DHT11
#define DHTTYPE DHT11   // Sensor tipe DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
  Serial.println("DHT11 siap di GPIO19");
}

void loop() {
  // Baca suhu dan kelembaban
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // default Celcius

  // Cek apakah pembacaan berhasil
  if (isnan(h) || isnan(t)) {
    Serial.println("Gagal membaca sensor DHT11!");
  } else {
    Serial.print("Kelembaban: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Suhu: ");
    Serial.print(t);
    Serial.println(" °C");
  }

  delay(2000); // baca setiap 2 detik
}
