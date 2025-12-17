#include <Arduino.h>
#include <Ticker.h>

#define ADC_PIN 34
#define DAC_PIN 25

const float Fs = 1000.0; // 1 kHz sampling
const float Fc = 0.5;   // cutoff 0.5 Hz

// 2nd-order LPF coefficients
float a0_2, a1_2, a2_2, b1_2, b2_2;
volatile float x1_2 = 0, x2_2 = 0;
volatile float y1_2 = 0, y2_2 = 0;

// 1st-order LPF coefficients
float alpha_1;
volatile float y1_1 = 0;

Ticker sampler;

void calculateCoefficients() {
  // 2nd-order Butterworth
  float K = tan(PI*Fc/Fs);
  float Q = 1.0 / sqrt(2.0);
  float norm = 1.0 / (1.0 + K/Q + K*K);
  a0_2 = K*K*norm;
  a1_2 = 2*a0_2;
  a2_2 = a0_2;
  b1_2 = 2*(K*K-1)*norm;
  b2_2 = (1-K/Q+K*K)*norm;

  // 1st-order LPF for cascading
  alpha_1 = K / (1 + K);
}

void onSample() {
  float x0 = analogRead(ADC_PIN)/4095.0;

  // 2nd-order stage
  float y2_stage = a0_2*x0 + a1_2*x1_2 + a2_2*x2_2 - b1_2*y1_2 - b2_2*y2_2;
  x2_2 = x1_2; x1_2 = x0;
  y2_2 = y1_2; y1_2 = y2_stage;

  // 1st-order stage (cascade)
  float y3_stage = alpha_1*y2_stage + (1-alpha_1)*y1_1;
  y1_1 = y3_stage;

  // Scale to DAC
  float dacVoltage = 1.5 + y3_stage * 1.0;
  dacVoltage = constrain(dacVoltage, 0.0, 3.3);
  dacWrite(DAC_PIN, (uint8_t)(dacVoltage/3.3*255));
}

void setup() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  dacWrite(DAC_PIN, 127);

  calculateCoefficients();
  sampler.attach_ms(1, onSample);
}

void loop() {
  // nothing
}
