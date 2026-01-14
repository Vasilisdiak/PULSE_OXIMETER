#include <Arduino.h>

/* ================= PIN DEFINITIONS ================= */
#define PIN_RED_LED   19      // RED LED always ON
#define PIN_ADC_DC    34      // DC level
#define PIN_ADC_AC    35      // AC amplified signal

/* ================= FILTER ================= */
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
LowPass dcFilter(0.01);      // very slow DC tracking
LowPass acFilter(0.2);       // smooth AC waveform

float ac_baseline = 0;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_RED_LED, OUTPUT);

  // RED always ON (100%)
  digitalWrite(PIN_RED_LED, HIGH);

  Serial.println("RED LED ALWAYS ON - EXPERIMENTAL MODE");
}

/* ================= LOOP ================= */
void loop() {

  int rawDC = analogRead(PIN_ADC_DC);
  int rawAC = analogRead(PIN_ADC_AC);

  // Convert to voltage (optional)
  float vDC = rawDC * (3.3 / 4095.0);
  float vAC = rawAC * (3.3 / 4095.0);

  // Slow DC average
  float dc = dcFilter.step(rawDC);

  // Remove AC offset (simple high-pass)
  ac_baseline = 0.995 * ac_baseline + 0.005 * rawAC;
  float pulse = rawAC - ac_baseline;

  // Smooth AC waveform
  float cleanAC = acFilter.step(pulse);

  // Plot / debug
  Serial.print("DC_raw:"); Serial.print(rawDC);
  Serial.print(" DC_V:"); Serial.print(vDC, 3);
  Serial.print(" AC_clean:"); Serial.println(cleanAC);

  delay(5);   // ~200 Hz output
}
