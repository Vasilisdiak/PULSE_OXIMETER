#include "driver/rmt.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// =============================================================
// -------------------- LCD CONFIGURATION ----------------------
// =============================================================
// Default I2C Address is usually 0x27 or 0x3F
LiquidCrystal_I2C lcd(0x27, 16, 2);  

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================
#define PIN_IR_WIN    23 
#define PIN_IR_PWM    15 
#define PIN_RED_WIN   19 
#define PIN_RED_PWM   14 

// HARDWARE CONFIGURATION
#define PIN_ADC_DC    34  // S_ph: Raw DC Level -> Used for AGC
#define PIN_ADC_AC    35  // G*S_ac: Amplified AC Pulse -> Used for Heartbeat

// =============================================================
// ------------------- TIMING SETTINGS -------------------------
// =============================================================
#define FSM_TICK_US     250   // Sample at 250us (Middle of pulse)
#define RMT_CLK_DIV     80    
#define CARRIER_PERIOD  20    
#define NUM_PULSES      25    

#define CH_IR_WIN   RMT_CHANNEL_0
#define CH_IR_PWM   RMT_CHANNEL_1
#define CH_RED_WIN  RMT_CHANNEL_2
#define CH_RED_PWM  RMT_CHANNEL_3

// =============================================================
// ------------------ GLOBAL VARIABLES -------------------------
// =============================================================
rmt_item32_t items_window[2];          
rmt_item32_t items_pwm[NUM_PULSES + 1]; 

volatile int fsm_state = 0;
hw_timer_t * timer = NULL;
int current_duty_percent = 85; 

// Storage for BOTH Pins
volatile int ir_dc_val = 0;
volatile int ir_ac_val = 0;
volatile int red_dc_val = 0;
volatile int red_ac_val = 0;
volatile bool newData = false;

// Final Results
float final_BPM = 0;
float final_SpO2 = 0;

// =============================================================
<<<<<<< HEAD
// ------------------- SMART FILTERS ---------------------------
=======
// ------------------- SIGNAL SMOOTHING ------------------------
>>>>>>> a8c64446505f850208477453a6c796bdf579c37e
// =============================================================
// We still need a little smoothing because raw ADC data is jittery
class LowPass {
    float alpha, val;
    public:
        LowPass(float a) { alpha = a; val = 0; }
        float step(float x) {
            val = (alpha * x) + ((1.0 - alpha) * val);
            return val;
        }
};

<<<<<<< HEAD
class SmartEnvelope {
    float val;
    float attack;  
    float release; 
    public:
        SmartEnvelope(float att, float rel) { 
            val = 0; attack = att; release = rel; 
        }
        void reset() { val = 0; }
        
        float step(float ac_input) {
            float abs_input = abs(ac_input);
            if (abs_input > val) val = val + attack * (abs_input - val);
            else val = val - release * (val - abs_input);
            return val * 2.0; 
        }
};

// Filters
LowPass agc_filter_IR(0.05);   
LowPass agc_filter_Red(0.05);
LowPass ac_center_IR(0.001);     
LowPass ac_center_Red(0.001);
SmartEnvelope env_IR(0.05, 0.0003); 
SmartEnvelope env_Red(0.05, 0.0003);
=======
// Filters for Signal Processing
LowPass avgDC_IR(0.01);    // Slow average for IR DC
LowPass avgDC_Red(0.01);   // Slow average for Red DC
LowPass smoothAC_IR(0.2);  // Fast smoothing for IR Heartbeat
LowPass smoothAC_Red(0.2); // Fast smoothing for Red Heartbeat
LowPass avgMag_IR(0.02);   // Magnitude tracker
LowPass avgMag_Red(0.02);  // Magnitude tracker
>>>>>>> a8c64446505f850208477453a6c796bdf579c37e

// =============================================================
// --------------- RMT & INTERRUPT SETUP -----------------------
// =============================================================
void setDutyCycle(int dutyPercent) {
  if (dutyPercent < 5) dutyPercent = 5;
  if (dutyPercent > 90) dutyPercent = 90;
  int high_us = (CARRIER_PERIOD * dutyPercent) / 100;
  int low_us  = CARRIER_PERIOD - high_us;
  if (high_us < 1) high_us = 1; if (low_us < 1) low_us = 1;
  for (int i = 0; i < NUM_PULSES; i++) items_pwm[i] = {{{ (uint16_t)high_us, 1, (uint16_t)low_us, 0 }}};
  items_pwm[NUM_PULSES] = {{{ 0, 0, 0, 0 }}};
  items_window[0] = {{{ 500, 1, 0, 0 }}}; items_window[1] = {{{ 0, 0, 0, 0 }}};
}

void setupRMT() {
  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX; config.clk_div = RMT_CLK_DIV; config.mem_block_num = 1;
  config.tx_config.loop_en = false; config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true; config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.channel = CH_IR_WIN;  config.gpio_num = (gpio_num_t)PIN_IR_WIN; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
  config.channel = CH_IR_PWM;  config.gpio_num = (gpio_num_t)PIN_IR_PWM; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
  config.channel = CH_RED_WIN; config.gpio_num = (gpio_num_t)PIN_RED_WIN; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
  config.channel = CH_RED_PWM; config.gpio_num = (gpio_num_t)PIN_RED_PWM; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
}

void IRAM_ATTR onTimerISR() {
  switch (fsm_state) {
    case 0: // START IR
      rmt_write_items(CH_IR_WIN, items_window, 2, false);
      rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES + 1, false); break;
      
    case 1: // SAMPLE IR (Mid-Point)
      // Read BOTH pins now
      ir_dc_val = analogRead(PIN_ADC_DC); 
      ir_ac_val = analogRead(PIN_ADC_AC); 
      break;
      
    case 4: // START RED
      rmt_write_items(CH_RED_WIN, items_window, 2, false);
      rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES + 1, false); break;
      
    case 5: // SAMPLE RED (Mid-Point)
      // Read BOTH pins now
      red_dc_val = analogRead(PIN_ADC_DC); 
      red_ac_val = analogRead(PIN_ADC_AC);
      newData = true; 
      break;
  }
  fsm_state = (fsm_state + 1) % 8;
}

// =============================================================
// ----------------------- MAIN SETUP --------------------------
// =============================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
  
  // SETUP BOTH PINS
  pinMode(PIN_ADC_DC, INPUT);
  pinMode(PIN_ADC_AC, INPUT);
  
  // LCD SETUP
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Booting..");

  setupRMT();
  setDutyCycle(current_duty_percent);
  timer = timerBegin(1000000); timerAttachInterrupt(timer, &onTimerISR);
  timerAlarm(timer, FSM_TICK_US, true, 0);
<<<<<<< HEAD
  Serial.println("System Running: LCD Active.");
=======
  Serial.println("System Running. Dual ADC Mode.");
>>>>>>> a8c64446505f850208477453a6c796bdf579c37e
}

// =============================================================
// ----------------------- MAIN LOOP ---------------------------
// =============================================================
void loop() {
  static unsigned long last_beat_time = 0;
  static float bpm_buffer[5];
  static int bpm_idx = 0;
  static unsigned long last_spo2_calc = 0;

  if (newData) {
    newData = false;

<<<<<<< HEAD
    // 1. Convert to Millivolts
    float Feedback_IR_mV = raw_feedback_IR * (3300.0 / 4095.0);   
    float Feedback_Red_mV = raw_feedback_Red * (3300.0 / 4095.0); 
    float Sig_IR_mV = raw_sig_IR * (3300.0 / 4095.0);   
    float Sig_Red_mV = raw_sig_Red * (3300.0 / 4095.0); 

    if (first_run) {
        ac_center_IR.set(Sig_IR_mV);
        ac_center_Red.set(Sig_Red_mV);
        first_run = false;
    }

    // 2. AGC LOGIC
    float FB_IR_Smooth = agc_filter_IR.step(Feedback_IR_mV);
    float FB_Red_Smooth = agc_filter_Red.step(Feedback_Red_mV);
    float TARGET = 1500.0; 
    float WINDOW = 50.0;

    if (FB_IR_Smooth > (TARGET + WINDOW) && duty_IR > 5) duty_IR--;
    else if (FB_IR_Smooth < (TARGET - WINDOW) && duty_IR < 90) duty_IR++;

    if (FB_Red_Smooth > (TARGET + WINDOW) && duty_Red > 5) duty_Red--;
    else if (FB_Red_Smooth < (TARGET - WINDOW) && duty_Red < 90) duty_Red++;

    if (duty_Red >= 90 && FB_Red_Smooth < (TARGET - WINDOW)) {
       if (FB_IR_Smooth > FB_Red_Smooth) duty_IR--; 
    }
    else if (duty_IR >= 90 && FB_IR_Smooth < (TARGET - WINDOW)) {
       if (FB_Red_Smooth > FB_IR_Smooth) duty_Red--;
    }

    // 3. SpO2 Processing
    float center_IR = ac_center_IR.step(Sig_IR_mV);
    float center_Red = ac_center_Red.step(Sig_Red_mV);
    float AC_IR = Sig_IR_mV - center_IR;
    float AC_Red = Sig_Red_mV - center_Red;

    float P2P_IR = env_IR.step(AC_IR);
    float P2P_Red = env_Red.step(AC_Red);

    // 4. BPM
    if (AC_IR > 50.0 && !beat_latch) {
        beat_latch = true;
        unsigned long now = millis();
        unsigned long delta = now - last_beat_time;
        if (delta > 270) {
            final_BPM = 60000.0 / delta;
            last_beat_time = now;
        }
=======
    // --- 1. AGC LOGIC (Using DC Pin 34) ---
    // This keeps the DC offset at ~1.45V so the AC amplifier works best
    float voltageDC = ir_dc_val * (3.3 / 4095.0);
    bool changed = false;
    
    // Target: 1.4V - 1.5V
    if (voltageDC > 1.5 && current_duty_percent > 5) { current_duty_percent--; changed = true; }
    else if (voltageDC < 1.4 && current_duty_percent < 95) { current_duty_percent++; changed = true; }
    if (changed) setDutyCycle(current_duty_percent);

    // --- 2. SIGNAL PROCESSING ---
    // Get smoothed DC baselines (from Pin 34)
    float dc_IR = avgDC_IR.step(ir_dc_val);
    float dc_Red = avgDC_Red.step(red_dc_val);

    // Get smoothed AC Pulse (from Hardware Pin 35)
    // NOTE: Pin 35 is centered around Vref/2 usually. 
    // We remove the static average to just get the "wiggle"
    static float ac_baseline_ir = 0;
    static float ac_baseline_red = 0;
    
    // Simple high-pass to center the hardware AC signal at 0
    ac_baseline_ir = (0.95 * ac_baseline_ir) + (0.05 * ir_ac_val);
    ac_baseline_red = (0.95 * ac_baseline_red) + (0.05 * red_ac_val);
    
    float pulse_IR = ir_ac_val - ac_baseline_ir;
    float pulse_Red = red_ac_val - ac_baseline_red;

    // Smooth the pulse for plotting
    float clean_IR = smoothAC_IR.step(pulse_IR);
    float clean_Red = smoothAC_Red.step(pulse_Red);

    // Track Magnitude for SpO2 Calc (Mean Absolute Deviation)
    float mag_IR = avgMag_IR.step(abs(pulse_IR));
    float mag_Red = avgMag_Red.step(abs(pulse_Red));

    // --- 3. BPM CALCULATION ---
    // Threshold crossing detection on the IR Pulse
    // If pulse rises above 2x the average magnitude, count a beat
    if (clean_IR > mag_IR * 1.5 && (millis() - last_beat_time > 300)) {
       // Debounce: ensure pulse is going UP
       if (clean_IR > bpm_buffer[bpm_idx]) { // simple check
           unsigned long time_now = millis();
           float instant_bpm = 60000.0 / (time_now - last_beat_time);
           last_beat_time = time_now;
    
           // Average last 5 beats
           bpm_buffer[bpm_idx] = instant_bpm;
           bpm_idx = (bpm_idx + 1) % 5;
           float bpm_sum = 0; for(int i=0; i<5; i++) bpm_sum += bpm_buffer[i];
           final_BPM = bpm_sum / 5.0;
       }
>>>>>>> a8c64446505f850208477453a6c796bdf579c37e
    }

<<<<<<< HEAD
    // 5. Output Handler (Serial + LCD)
    static unsigned long last_update = 0;
    
    // Update every 100ms for Math/Serial
    if (millis() - last_update > 100) {
        last_update = millis();

        if (P2P_IR > 20.0) {
            float Norm_Red = P2P_Red / STAGE2_BIAS_MV;
            float Norm_IR = P2P_IR / STAGE2_BIAS_MV;
            float R = Norm_Red / Norm_IR;
            float calc = 110.0 - (25.0 * R);
            
            if (calc > 100) calc = 100;
            if (calc < 60) calc = 60;
            
            final_SpO2 = (final_SpO2 * 0.9) + (calc * 0.1);

            // SERIAL
            Serial.print("BPM:"); Serial.print(final_BPM);
            Serial.print(" SpO2:"); Serial.print(final_SpO2);
            Serial.print(" Pulse_Wave:"); Serial.println(AC_IR);
            
            // LCD UPDATE (Every 500ms to avoid flicker)
            static unsigned long lcd_timer = 0;
            if (millis() - lcd_timer > 500) {
                lcd_timer = millis();
                lcd.clear();
                
                // Row 0: SpO2
                lcd.setCursor(0, 0);
                lcd.print("SPO2 = ");
                lcd.print((int)final_SpO2);
                lcd.print("%");

                // Row 1: BPM
                lcd.setCursor(0, 1);
                lcd.print("Heart rate = ");
                lcd.print((int)final_BPM);
            }
            
        } else {
            Serial.println("Signal_Low"); 
            final_BPM = 0;
            final_SpO2 = 0;
            if(duty_IR < 85) duty_IR++;
            if(duty_Red < 85) duty_Red++;

            // LCD Message for No Finger
            static unsigned long lcd_timer = 0;
            if (millis() - lcd_timer > 500) {
                lcd_timer = millis();
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Place Finger");
                lcd.setCursor(0, 1);
                lcd.print("Signal Low...");
            }
        }
=======
    // --- 4. SpO2 CALCULATION ---
    // Formula: R = (AC_red / DC_red) / (AC_ir / DC_ir)
    if (millis() - last_spo2_calc > 500) {
      last_spo2_calc = millis();
      
      // Safety: Only calc if finger is present (DC > 0.5V)
      if (voltageDC > 0.5) {
          float Ratio = (mag_Red / dc_Red) / (mag_IR / dc_IR);
          
          // Empirical Curve: SpO2 = 110 - 25 * R (Adjusted for typical hardware)
          // You may need to tune these numbers: 110 and 25
          float calc_spo2 = 110.0 - (25.0 * Ratio);
          
          // Clamp
          if(calc_spo2 > 100) calc_spo2 = 100;
          if(calc_spo2 < 70) calc_spo2 = 70;
          
          final_SpO2 = (final_SpO2 * 0.9) + (calc_spo2 * 0.1);
      } else {
          final_BPM = 0; final_SpO2 = 0;
      }
>>>>>>> a8c64446505f850208477453a6c796bdf579c37e
    }

    // --- 5. PLOTTING ---
    Serial.print("BPM:"); Serial.print(final_BPM);
    Serial.print(" SpO2:"); Serial.print(final_SpO2);
    // Plot the Hardware AC Signal
    Serial.print(" Pulse_Wave:"); Serial.println(clean_IR); 
  }
  delay(5);
}