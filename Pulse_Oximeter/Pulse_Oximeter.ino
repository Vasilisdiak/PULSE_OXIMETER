#include "driver/rmt.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================
#define PIN_IR_WIN    22 
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

// --- INDEPENDENT DUTY CYCLES ---
int duty_IR = 85; 
int duty_Red = 85;

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
// ------------------- SIGNAL SMOOTHING ------------------------
// =============================================================
class LowPass {
    float alpha, val;
    public:
        LowPass(float a) { alpha = a; val = 0; }
        float step(float x) {
            val = (alpha * x) + ((1.0 - alpha) * val);
            return val;
        }
};

// Filters
LowPass avgDC_IR(0.05);    // Fast DC tracking for AGC
LowPass avgDC_Red(0.05);   
LowPass smoothAC_IR(0.2);  
LowPass smoothAC_Red(0.2); 
LowPass avgMag_IR(0.02);   
LowPass avgMag_Red(0.02);  

// =============================================================
// --------------- RMT & INTERRUPT SETUP -----------------------
// =============================================================
void loadBufferWithDuty(int dutyPercent) {
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
      loadBufferWithDuty(duty_IR);
      rmt_write_items(CH_IR_WIN, items_window, 2, false);
      rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES + 1, false); break;
      
    case 1: // SAMPLE IR
      ir_dc_val = analogRead(PIN_ADC_DC); 
      ir_ac_val = analogRead(PIN_ADC_AC); 
      break;
      
    case 4: // START RED
      loadBufferWithDuty(duty_Red);
      rmt_write_items(CH_RED_WIN, items_window, 2, false);
      rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES + 1, false); break;
      
    case 5: // SAMPLE RED
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
  pinMode(PIN_ADC_DC, INPUT);
  pinMode(PIN_ADC_AC, INPUT);
  
  setupRMT();
  loadBufferWithDuty(85);
  
  timer = timerBegin(1000000); timerAttachInterrupt(timer, &onTimerISR);
  timerAlarm(timer, FSM_TICK_US, true, 0);
  Serial.println("System Running. Dual Independent AGC (Target: 1.8V).");
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

    // --- 1. SIGNAL PROCESSING ---
    float vDC_IR = ir_dc_val * (3.3 / 4095.0);
    float vDC_Red = red_dc_val * (3.3 / 4095.0);

    float dc_IR_smooth = avgDC_IR.step(ir_dc_val);
    float dc_Red_smooth = avgDC_Red.step(red_dc_val);

    // --- 2. INDEPENDENT DUAL AGC (TARGET: 1.8V) ---
    // If voltage > 1.85V -> Too bright, reduce duty
    // If voltage < 1.75V -> Too dim, increase duty
    
    // --> Control IR LED
    if (vDC_IR > 1 && duty_IR > 2) duty_IR--;      
    else if (vDC_IR < 0.95 && duty_IR < 90) duty_IR++; 

    // --> Control RED LED
    if (vDC_Red > 1 && duty_Red > 2) duty_Red--;      
    else if (vDC_Red < 0.95 && duty_Red < 90) duty_Red++; 

    // --- 3. AC SIGNAL PROCESSING ---
    static float ac_baseline_ir = 0;
    static float ac_baseline_red = 0;
    
    ac_baseline_ir = (0.95 * ac_baseline_ir) + (0.05 * ir_ac_val);
    ac_baseline_red = (0.95 * ac_baseline_red) + (0.05 * red_ac_val);
    
    float pulse_IR = ir_ac_val - ac_baseline_ir;
    float pulse_Red = red_ac_val - ac_baseline_red;

    float clean_IR = smoothAC_IR.step(pulse_IR);
    float clean_Red = smoothAC_Red.step(pulse_Red);

    float mag_IR = avgMag_IR.step(abs(pulse_IR));
    float mag_Red = avgMag_Red.step(abs(pulse_Red));

    // --- 4. BPM DETECTION ---
    if (clean_IR > mag_IR * 1.5 && (millis() - last_beat_time > 300)) {
       if (clean_IR > 20) { 
           unsigned long time_now = millis();
           float instant_bpm = 60000.0 / (time_now - last_beat_time);
           last_beat_time = time_now;
    
           if(instant_bpm > 40 && instant_bpm < 180) {
             bpm_buffer[bpm_idx] = instant_bpm;
             bpm_idx = (bpm_idx + 1) % 5;
             float bpm_sum = 0; for(int i=0; i<5; i++) bpm_sum += bpm_buffer[i];
             final_BPM = bpm_sum / 5.0;
           }
       }
    }

    // --- 5. SpO2 CALCULATION ---
    if (millis() - last_spo2_calc > 500) {
      last_spo2_calc = millis();
      
      // Ensure we have a valid signal before calculating
      if (vDC_IR > 0.5) {
          float Ratio = (mag_Red / dc_Red_smooth) / (mag_IR / dc_IR_smooth);
          float calc_spo2 = 110.0 - (25.0 * Ratio);
          
          if(calc_spo2 > 100) calc_spo2 = 100;
          if(calc_spo2 < 70) calc_spo2 = 70;
          
          final_SpO2 = (final_SpO2 * 0.9) + (calc_spo2 * 0.1);
      } else {
          final_BPM = 0; final_SpO2 = 0;
      }
    }

    // --- 6. PLOTTING ---
    Serial.print("BPM:"); Serial.print(final_BPM);
    Serial.print(" SpO2:"); Serial.print(final_SpO2);
    
    // Duty Cycle Feedback
    Serial.print(" IR_Duty:"); Serial.print(duty_IR);
    Serial.print(" Red_Duty:"); Serial.print(duty_Red);

    Serial.print(" Pulse_Wave:"); Serial.println(clean_IR); 
  }
  delay(5);
}