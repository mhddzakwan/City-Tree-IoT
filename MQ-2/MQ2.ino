#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// Nilai resistansi beban (RL) pada MQ2 (biasanya 5kΩ)
const float RL = 5.0; // dalam kilo ohm

// Nilai R0 hasil kalibrasi di udara bersih (diisi setelah kalibrasi)
float R0 = 10.0; // contoh nilai awal (akan diubah setelah kalibrasi)

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!ads.begin()) {
    Serial.println("Gagal deteksi ADS1115!");
    while (1);
  }
  
  // atur penguatan agar range bacaan 0–6.144V
  ads.setGain(GAIN_TWOTHIRDS); 
  Serial.println("ADS1115 siap. Membaca MQ2...");
}

void loop() {
  // Baca data ADC channel 0
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.1875 / 1000; // konversi ke volt (0.1875 mV per bit)
  
  // Hitung resistansi sensor (RS)
  float RS = ((5.0 * RL) / voltage) - RL; // berdasarkan voltage divider MQ2
  
  // Hitung rasio Rs/R0
  float ratio = RS / R0;
  
  // Estimasi konsentrasi CO (ppm)
  // Berdasarkan kurva MQ2 untuk CO: log(ppm) = (log(ratio) - b) / m
  // Dari datasheet: m ≈ -0.38, b ≈ 0.48
  float ppm = pow(10, ((log10(ratio) - 0.48) / -0.38));

  if (ppm > 1000) ppm = 1000;
  
  Serial.print("Tegangan: ");
  Serial.print(voltage, 3);
  Serial.print(" V | Rs/R0: ");
  Serial.print(ratio, 2);
  Serial.print(" | CO: ");
  Serial.print(ppm, 1);
  Serial.println(" ppm");

  delay(2000);
}
