#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Alamat I2C, 20 kolom, 4 baris
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  // Inisialisasi I2C pada pin alternatif ESP32
  Wire.begin(25, 26); // SDA=GPIO25, SCL=GPIO26

  lcd.begin();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("ESP32 Tersambung!");
  
  lcd.setCursor(0, 1);
  lcd.print("LCD 20x4 I2C Aktif");

  lcd.setCursor(0, 2);
  lcd.print("Monitoring Sensor:");

  lcd.setCursor(0, 3);
  lcd.print("Semua OK!");
}

void loop() {
  // Tambahkan update data sensor di sini
}
