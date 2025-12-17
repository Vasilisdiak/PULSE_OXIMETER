#include <Arduino.h>
#include <Ticker.h>

#define ADC_PIN 34
#define DAC_PIN 25

const float Fs = 1000.0;  // 1 kHz sampling
const float fcl = 0.5;    // low cutoff 0.5 Hz
const float fch = 5.0;    // high cutoff 5 Hz

// 1st-order HPF coefficients
float alpha_hp;
volatile float x_hp = 0, y_hp_old = 0;

// 2nd-order LPF coefficients (Butterworth)
float a0_lp, a1_lp, a2_lp, b1_lp, b2_lp;
volatile float x1_lp = 0, x2_lp = 0;
volatile float y1_lp = 0, y2_lp = 0;

Ticker sampler;

void calculateCoefficients() {
  // HPF 1st order
  alpha_hp = Fs / (Fs + 2.0 * PI * fcl);

  // LPF 2nd order Butterworth
  float K = tan(PI * fch / Fs);
  float Q = 1.0 / sqrt(2.0);
  float norm = 1.0 / (1.0 + K / Q + K * K);
  a0_lp = K * K * norm;
  a1_lp = 2.0 * a0_lp;
  a2_lp = a0_lp;
  b1_lp = 2.0 * (K * K - 1.0) * norm;
  b2_lp = (1.0 - K / Q + K * K) * norm;
}

void onSample() {
  // ADC input, normalized 0..1
  float x0 = analogRead(ADC_PIN) / 4095.0;

  // 1st-order HPF
  float y_hp = alpha_hp * (y_hp_old + x0 - x_hp);
  x_hp = x0;
  y_hp_old = y_hp;

  // 2nd-order LPF (Butterworth)
  float y_lp = a0_lp * y_hp + a1_lp * x1_lp + a2_lp * x2_lp - b1_lp * y1_lp - b2_lp * y2_lp;

  // Update state for LPF
  x2_lp = x1_lp;
  x1_lp = y_hp;
  y2_lp = y1_lp;
  y1_lp = y_lp;

  // DAC output, scale ±1V around 1.5V
  float dacVoltage = 1.5 + y_lp * 1.0;
  dacVoltage = constrain(dacVoltage, 0.0, 3.3);
  dacWrite(DAC_PIN, (uint8_t)(dacVoltage / 3.3 * 255));
}

void setup() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  dacWrite(DAC_PIN, 127);  // start ~1.5V

  calculateCoefficients();

  // Sample every 1ms → 1 kHz
  sampler.attach_ms(1, onSample);
}

void loop() {
  // nothing needed, sampling handled by Ticker
}
