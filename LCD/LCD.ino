#include <LiquidCrystal.h>

const int RS = 14;
const int E  = 12;
const int D4 = 27;
const int D5 = 26;
const int D6 = 25;
const int D7 = 33;

// Δημιουργία αντικειμένου LiquidCrystal_ESP32
LiquidCrystal My_LCD(RS, E, D4, D5, D6, D7);

void setup() 
{
  // Initialize The LCD. Parameters: [ Columns, Rows ] 
  My_LCD.begin(16, 2); 
  // Clears The LCD Display 
  My_LCD.clear();
 
  // Display The First Message In Home Position (0, 0) 
  My_LCD.print("SPO2 = 100%"); 
  // Set The Cursor Position To: [ Col 5, Row 1] 
  // The Next Message Will Start From The 6th Char Position in The 2nd Row 
  // Note: 1st Row Has An Index of 0, The 2nd Row Has An Index of 1 
  My_LCD.setCursor(0, 1); 
  // Display The Second Message In Position (5, 1) 
  My_LCD.print("Heart Pulse = 62");
}
 
void loop() 
{
  // Do Nothing...
}