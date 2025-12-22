#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/dac.h"
#include <LiquidCrystal.h>
#include <Ticker.h>

// =============================================================
// -------------------- PIN ASSIGNMENTS ------------------------
// =============================================================

// --- ANALOG I/O ---
#define ADC_PIN        34
#define DAC_PIN_GPIO   25  // DAC Channel 1

// --- SIGNAL GENERATION (RMT) ---
#define PIN_PIR        5
#define PIN_CIR        15
#define PIN_PR         19
#define PIN_CR         14

// --- LCD PINS (REMAPPED TO AVOID CONFLICTS) ---
// Note: Original RS was 14 (Conflict with CR), D6 was 25 (Conflict with DAC)
const int RS = 13; 
const int E  = 12;
const int D4 = 27;
const int D5 = 26;
const int D6 = 18; // Changed from 25
const int D7 = 33;

LiquidCrystal My_LCD(RS, E, D4, D5, D6, D7);

// =============================================================
// -------------------- SIGNAL CONSTANTS -----------------------
// =============================================================
const int PERIOD_US       = 2000; // 2ms Total Frame (500Hz)
const int DUTY_US         = 500;
const int CARRIER_PERIOD  = 50;   // 20kHz
const int NUM_PULSES      = 10;
const int START_DELAY_US  = 1000;
const int END_DELAY_US    = 500;
const int HIGH_US         = CARRIER_PERIOD / 2;
const int LOW_US          = CARRIER_PERIOD / 2;

// RMT Buffers
const int MAX_CYCLES = DUTY_US / CARRIER_PERIOD;
rmt_item32_t pir_item[1];
rmt_item32_t cir_items[MAX_CYCLES + 1];
rmt_item32_t pr_items[2];
rmt_item32_t cr_items[NUM_PULSES + 2]; // +2 to handle pre-delay and post-delay

// =============================================================
// -------------------- FILTER CONSTANTS -----------------------
// =============================================================
const float Fs = 1000.0;  // 1 kHz sampling
const float fcl = 0.5;    // HPF Cutoff
const float fch = 5.0;    // LPF Cutoff

// Filter Variables
float alpha_hp;
volatile float x_hp = 0, y_hp_old = 0;
float a0_lp, a1_lp, a2_lp, b1_lp, b2_lp;
volatile float x1_lp = 0, x2_lp = 0;
volatile float y1_lp = 0, y2_lp = 0;

// Sampling Timer
Ticker sampler;

// =============================================================
// -------------------- SETUP FUNCTIONS ------------------------
// =============================================================

void calculateFilterCoefficients() {
  // 1. HPF 1st order
  alpha_hp = Fs / (Fs + 2.0 * PI * fcl);

  // 2. LPF 2nd order Butterworth
  float K = tan(PI * fch / Fs);
  float Q = 1.0 / sqrt(2.0);
  float norm = 1.0 / (1.0 + K / Q + K * K);
  
  a0_lp = K * K * norm;
  a1_lp = 2.0 * a0_lp;
  a2_lp = a0_lp;
  b1_lp = 2.0 * (K * K - 1.0) * norm;
  b2_lp = (1.0 - K / Q + K * K) * norm;
}

void setupRMT() {
    // ---------------- PIR (Channel 0) ----------------
    // Simple Pulse: 500us HIGH, 1500us LOW
    rmt_config_t cfg_pir = {};
    cfg_pir.channel = RMT_CHANNEL_0;
    cfg_pir.gpio_num = (gpio_num_t)PIN_PIR;
    cfg_pir.mem_block_num = 1;
    cfg_pir.clk_div = 80; // 1us tick
    cfg_pir.tx_config.loop_en = true; // HARDWARE LOOP ENABLED
    cfg_pir.tx_config.idle_output_en = true;
    cfg_pir.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pir.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pir);
    rmt_driver_install(cfg_pir.channel, 0, 0);

    pir_item[0].level0 = 1; pir_item[0].duration0 = DUTY_US;
    pir_item[0].level1 = 0; pir_item[0].duration1 = PERIOD_US - DUTY_US;
    rmt_write_items(RMT_CHANNEL_0, pir_item, 1, false);

    // ---------------- CIR (Channel 1) ----------------
    // Carrier Burst: 20kHz for 500us, then LOW
    rmt_config_t cfg_cir = {};
    cfg_cir.channel = RMT_CHANNEL_1;
    cfg_cir.gpio_num = (gpio_num_t)PIN_CIR;
    cfg_cir.mem_block_num = 2; // Needs more memory for pulses
    cfg_cir.clk_div = 80;
    cfg_cir.tx_config.loop_en = true;
    cfg_cir.tx_config.idle_output_en = true;
    cfg_cir.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_cir.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_cir);
    rmt_driver_install(cfg_cir.channel, 0, 0);

    for (int i = 0; i < MAX_CYCLES; i++) {
        cir_items[i].level0 = 1; cir_items[i].duration0 = HIGH_US;
        cir_items[i].level1 = 0; cir_items[i].duration1 = LOW_US;
    }
    // Fill remaining time in frame
    cir_items[MAX_CYCLES].level0 = 0; 
    cir_items[MAX_CYCLES].duration0 = PERIOD_US - DUTY_US; 
    cir_items[MAX_CYCLES].level1 = 0; 
    cir_items[MAX_CYCLES].duration1 = 0; // Terminator
    rmt_write_items(RMT_CHANNEL_1, cir_items, MAX_CYCLES + 1, false);

    // ---------------- PR (Channel 2) ----------------
    // Delayed Pulse: 1000us LOW, 500us HIGH, 500us LOW
    rmt_config_t cfg_pr = {};
    cfg_pr.channel = RMT_CHANNEL_2;
    cfg_pr.gpio_num = (gpio_num_t)PIN_PR;
    cfg_pr.mem_block_num = 1;
    cfg_pr.clk_div = 80;
    cfg_pr.tx_config.loop_en = true;
    cfg_pr.tx_config.idle_output_en = true;
    cfg_pr.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pr.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pr);
    rmt_driver_install(cfg_pr.channel, 0, 0);

    pr_items[0].level0 = 0; pr_items[0].duration0 = START_DELAY_US;
    pr_items[0].level1 = 1; pr_items[0].duration1 = 500;
    pr_items[1].level0 = 0; pr_items[1].duration0 = END_DELAY_US;
    pr_items[1].level1 = 0; pr_items[1].duration1 = 0; 
    rmt_write_items(RMT_CHANNEL_2, pr_items, 2, false);

    // ---------------- CR (Channel 3) ----------------
    // Delayed Carrier: 1000us LOW, 20kHz bursts, 500us LOW
    rmt_config_t cfg_cr = {};
    cfg_cr.channel = RMT_CHANNEL_3;
    cfg_cr.gpio_num = (gpio_num_t)PIN_CR;
    cfg_cr.mem_block_num = 2;
    cfg_cr.clk_div = 80;
    cfg_cr.tx_config.loop_en = true;
    cfg_cr.tx_config.idle_output_en = true;
    cfg_cr.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_cr.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_cr);
    rmt_driver_install(cfg_cr.channel, 0, 0);

    // 1. Initial Delay
    cr_items[0].level0 = 0; cr_items[0].duration0 = START_DELAY_US;
    cr_items[0].level1 = 1; cr_items[0].duration1 = HIGH_US; // Start first pulse
    
    // 2. Burst Pulses (Start from i=1 because i=0 was delay)
    for (int i = 1; i < NUM_PULSES; i++) {
        cr_items[i].level0 = 0; cr_items[i].duration0 = LOW_US;
        cr_items[i].level1 = 1; cr_items[i].duration1 = HIGH_US;
    }

    // 3. Final Delay
    cr_items[NUM_PULSES].level0 = 0; 
    cr_items[NUM_PULSES].duration0 = LOW_US; // Finish last pulse low
    cr_items[NUM_PULSES].level1 = 0; 
    cr_items[NUM_PULSES].duration1 = END_DELAY_US; 

    // Terminator
    cr_items[NUM_PULSES+1].level0 = 0; cr_items[NUM_PULSES+1].duration0 = 0;
    cr_items[NUM_PULSES+1].level1 = 0; cr_items[NUM_PULSES+1].duration1 = 0;
    
    rmt_write_items(RMT_CHANNEL_3, cr_items, NUM_PULSES + 2, false);
}

// =============================================================
// -------------------- INTERRUPT ROUTINE ----------------------
// =============================================================

// This runs every 1ms (1kHz)
void onSample() {
  // 1. Read ADC (Normalized 0.0 - 1.0)
  float x0 = analogRead(ADC_PIN) / 4095.0;

  // 2. Apply High Pass Filter
  float y_hp = alpha_hp * (y_hp_old + x0 - x_hp);
  x_hp = x0;
  y_hp_old = y_hp;

  // 3. Apply Low Pass Filter (Butterworth)
  float y_lp = a0_lp * y_hp + a1_lp * x1_lp + a2_lp * x2_lp - b1_lp * y1_lp - b2_lp * y2_lp;

  // Update State
  x2_lp = x1_lp;
  x1_lp = y_hp;
  y2_lp = y1_lp;
  y1_lp = y_lp;

  // 4. Output Filtered Signal to DAC for visualization
  // Scale result: 1.5V offset + signal
  float dacVoltage = 1.5 + y_lp * 1.0; 
  dacVoltage = constrain(dacVoltage, 0.0, 3.3);
  dacWrite(DAC_PIN_GPIO, (uint8_t)(dacVoltage / 3.3 * 255));
}

// =============================================================
// ----------------------- MAIN SETUP --------------------------
// =============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  // 1. Setup LCD
  My_LCD.begin(16, 2);
  My_LCD.clear();
  My_LCD.print("System Init...");

  // 2. Setup Analog/DAC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  dac_output_enable(DAC_CHANNEL_1);
  dacWrite(DAC_PIN_GPIO, 127); // Start mid-rail

  // 3. Calculate DSP Coefficients
  calculateFilterCoefficients();

  // 4. Start Signal Generation (RMT)
  // This runs in background using DMA/Hardware buffers
  setupRMT(); 

  // 5. Start Sampling Timer
  // Calls onSample every 1ms
  sampler.attach_ms(1, onSample);

  delay(1000);
  My_LCD.clear();
}

// =============================================================
// ----------------------- MAIN LOOP ---------------------------
// =============================================================

void loop() {
  // Update LCD every 500ms
  // We do NOT simulate Pulse Ox calculation here as it requires complex
  // peak detection logic not present in source files.
  // We display static or raw data as placeholders.

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 500) {
    lastUpdate = millis();

    // Just for visualization, we can print the current raw ADC value
    int rawVal = analogRead(ADC_PIN);
    
    My_LCD.clear();
    My_LCD.setCursor(0, 0);
    My_LCD.print("SPO2: -- %"); // Placeholder
    
    My_LCD.setCursor(0, 1);
    My_LCD.print("Raw ADC: ");
    My_LCD.print(rawVal);
  }
}