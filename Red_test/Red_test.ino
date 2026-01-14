#include <Arduino.h>

/* ================= PIN DEFINITIONS ================= */
// To turn the Red LED fully ON, we must enable both the Window and the Signal pins
#define PIN_RED_WIN   19      // The "Window" enable pin
#define PIN_RED_PWM   14      // The "PWM/Signal" pin

#define PIN_ADC_DC    34      // DC level input
#define PIN_ADC_AC    35      // AC amplified input

/* ================= FILTER CLASS ================= */
class LowPass {
  float alpha;
  float y;
public:
  LowPass(float a) : alpha(a), y(0) {}
  float step(float x) {
    y = alpha * x + (1.0 - alpha) * y;
    return y;
  }
};

/* ================= GLOBALS ================= */
// Filters
LowPass dcFilter(0.01);      // Heavy smoothing for DC
LowPass acFilter(0.2);       // Light smoothing for AC

float ac_baseline = 0;       // For removing DC offset from the AC pin

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  // ADC Config
  analogReadResolution(12);       // 0 - 4095
  analogSetAttenuation(ADC_11db); // Input range 0 - 3.1V

  // Configure LED Pins
  pinMode(PIN_RED_WIN, OUTPUT);
  pinMode(PIN_RED_PWM, OUTPUT);

  // --- ACTIVATE PERMANENT RED LIGHT ---
  // We write HIGH to both to simulate a 100% duty cycle
  digitalWrite(PIN_RED_WIN, HIGH);
  digitalWrite(PIN_RED_PWM, HIGH);

  Serial.println("TEST STARTED: Red Window & PWM set to HIGH (Continuous DC Light)");
}

/* ================= LOOP ================= */
void loop() {
  // 1. Read Raw Values
  int rawDC = analogRead(PIN_ADC_DC);
  int rawAC = analogRead(PIN_ADC_AC);

  // 2. Convert to Voltage (Optional, for reference)
  float vDC = rawDC * (3.3 / 4095.0);

  // 3. Filter DC Signal (Pin 34)
  float cleanDC = dcFilter.step(rawDC);

  // 4. Filter AC Signal (Pin 35)
  // First, find the average of the AC pin to treat it as "0"
  ac_baseline = (0.995 * ac_baseline) + (0.005 * rawAC);
  float pulse = rawAC - ac_baseline;
  
  // Smooth the result
  float cleanAC = acFilter.step(pulse);

  // 5. Serial Plotter Output
  // DC_raw: The actual raw reading from Pin 34
  // AC_clean: The centered "wiggle" from Pin 35
  Serial.print("DC_raw:");   Serial.print(rawDC);
  Serial.print(" AC_clean:"); Serial.println(cleanAC);

  delay(5); // Run at ~200Hz
}