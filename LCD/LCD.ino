#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---- LCD Settings ----
int lcdColumns = 16;
int lcdRows = 2;
// Αν δεν ξέρεις τη διεύθυνση, τρέξε το I2C scanner (συνήθως 0x27)
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

void setup() {
  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Clear display
  lcd.clear();

  // Display first message in first row
  lcd.setCursor(0, 0);
  lcd.print("SPO2 = 100%");

  // Display second message in second row
  lcd.setCursor(0, 1);
  lcd.print("Heart Pulse = 62");
}

void loop() {
  // Nothing to do, static display
}
