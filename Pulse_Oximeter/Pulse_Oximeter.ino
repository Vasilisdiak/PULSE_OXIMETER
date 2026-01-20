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

#define PIN_ADC_FEEDBACK  34  // AGC
#define PIN_ADC_IR_SIG    35  // Signal
#define PIN_ADC_RED_SIG   36  // Signal

// =============================================================
// ------------------- CONFIGURATION ---------------------------
// =============================================================
#define FSM_TICK_US       250   // 4000Hz
#define RMT_CLK_DIV       80    
#define CARRIER_PERIOD    20    
#define NUM_PULSES        25    

#define CH_IR_WIN   RMT_CHANNEL_0
#define CH_IR_PWM   RMT_CHANNEL_1
#define CH_RED_WIN  RMT_CHANNEL_2
#define CH_RED_PWM  RMT_CHANNEL_3

const float STAGE2_BIAS_MV = 1500.0; 

// =============================================================
// ------------------ GLOBAL VARIABLES -------------------------
// =============================================================
rmt_item32_t items_window[2];          
rmt_item32_t items_pwm[NUM_PULSES + 1]; 

volatile int fsm_state = 0;
hw_timer_t * timer = NULL;

int duty_IR = 85; 
int duty_Red = 85;

volatile int raw_feedback_IR = 0;   
volatile int raw_feedback_Red = 0;  
volatile int raw_sig_IR = 0;        
volatile int raw_sig_Red = 0;       
volatile bool newData = false;

float final_BPM = 0;
float final_SpO2 = 0;

// =============================================================
// ------------------- NEW SMART ENVELOPE ----------------------
// =============================================================

class LowPass {
    float alpha, val;
    public:
        LowPass(float a) { alpha = a; val = 0; }
        void set(float x) { val = x; } 
        float step(float x) {
            val = (alpha * x) + ((1.0 - alpha) * val);
            return val;
        }
};

// NEW CLASS: Replaces AmplitudeTracker
// Uses Asymmetrical Smoothing to handle Rise vs Fall naturally
class SmartEnvelope {
    float val;
    float attack;  // Speed when going UP
    float release; // Speed when going DOWN
    public:
        SmartEnvelope(float att, float rel) { 
            val = 0; 
            attack = att; 
            release = rel; 
        }
        
        void reset() { val = 0; }
        
        float step(float ac_input) {
            // We only care about the magnitude (absolute height)
            float abs_input = abs(ac_input);
            
            if (abs_input > val) {
                // RISING (Signal is bigger than tracker)
                // Use Attack speed. 
                // Formula: val = val + speed * (target - val)
                val = val + attack * (abs_input - val);
            } else {
                // FALLING (Signal is smaller than tracker)
                // Use Release speed.
                val = val - release * (val - abs_input);
            }
            
            // The P2P is roughly 2x the Envelope (Peak-to-Peak vs Peak)
            // We multiply by 2.0 to restore the full height number
            return val * 2.0; 
        }
};

// --- AGC FILTERS ---
LowPass agc_filter_IR(0.05);   
LowPass agc_filter_Red(0.05);

// --- SIGNAL CENTER FILTERS ---
// 0.001 is fast enough to track bias, slow enough for 1Hz
LowPass ac_center_IR(0.001);     
LowPass ac_center_Red(0.001);

// --- NEW TRACKERS ---
// Attack 0.05: Fast rise (~0.1s) but smooths noise spikes (No Overshoot)
// Release 0.0003: Slow fall to handle 1Hz gaps, but exponential drop (No "Stuck" feeling)
SmartEnvelope env_IR(0.05, 0.0003); 
SmartEnvelope env_Red(0.05, 0.0003);

// =============================================================
// --------------- RMT & INTERRUPT SETUP -----------------------
// =============================================================
void loadBufferWithDuty(int dutyPercent) {
  if (dutyPercent < 1) dutyPercent = 1;
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
    case 0: // IR FIRE
      loadBufferWithDuty(duty_IR);
      rmt_write_items(CH_IR_WIN, items_window, 2, false);
      rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES + 1, false); break;
    case 1: // IR SAMPLE
      raw_feedback_IR = analogRead(PIN_ADC_FEEDBACK); 
      raw_sig_IR = analogRead(PIN_ADC_IR_SIG);        
      break;
    case 4: // RED FIRE
      loadBufferWithDuty(duty_Red);
      rmt_write_items(CH_RED_WIN, items_window, 2, false);
      rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES + 1, false); break;
    case 5: // RED SAMPLE
      raw_feedback_Red = analogRead(PIN_ADC_FEEDBACK); 
      raw_sig_Red = analogRead(PIN_ADC_RED_SIG);       
      newData = true; break;
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
  
  pinMode(PIN_ADC_FEEDBACK, INPUT); 
  pinMode(PIN_ADC_IR_SIG, INPUT);   
  pinMode(PIN_ADC_RED_SIG, INPUT);  
  
  setupRMT();
  
  timer = timerBegin(1000000); timerAttachInterrupt(timer, &onTimerISR);
  timerAlarm(timer, FSM_TICK_US, true, 0);
  Serial.println("System Running: Smart Envelope Mode.");
}

// =============================================================
// ----------------------- MAIN LOOP ---------------------------
// =============================================================
void loop() {
  static unsigned long last_beat_time = 0;
  static bool beat_latch = false;
  static bool first_run = true; 

  if (newData) {
    newData = false;

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

    // 3. SpO2 LOGIC (Using Smart Envelope)
    
    // A. Center Signals
    float center_IR = ac_center_IR.step(Sig_IR_mV);
    float center_Red = ac_center_Red.step(Sig_Red_mV);
    
    float AC_IR = Sig_IR_mV - center_IR;
    float AC_Red = Sig_Red_mV - center_Red;

    // B. Calculate Envelope (Replaces P2P)
    // This will now rise smoothly (no overshoot) and fall exponentially (faster feel)
    float P2P_IR = env_IR.step(AC_IR);
    float P2P_Red = env_Red.step(AC_Red);

    // C. BPM
    if (AC_IR > 50.0 && !beat_latch) {
        beat_latch = true;
        unsigned long now = millis();
        unsigned long delta = now - last_beat_time;
        if (delta > 270) {
            final_BPM = 60000.0 / delta;
            last_beat_time = now;
        }
    }
    if (AC_IR < -50.0) beat_latch = false;

    // D. Output
    static unsigned long last_calc = 0;
    if (millis() - last_calc > 100) {
        last_calc = millis();

        if (P2P_IR > 20.0) {
            float Norm_Red = P2P_Red / STAGE2_BIAS_MV;
            float Norm_IR = P2P_IR / STAGE2_BIAS_MV;
            float R = Norm_Red / Norm_IR;

            float calc = 110.0 - (25.0 * R);
            if (calc > 100) calc = 100;
            if (calc < 60) calc = 60;
            
            final_SpO2 = (final_SpO2 * 0.9) + (calc * 0.1);

            Serial.print("BPM:"); Serial.print(final_BPM);
            Serial.print(" SpO2:"); Serial.print(final_SpO2);
           // Serial.print(" Pulse_Wave:"); Serial.print(AC_IR);
            
            // Check responsiveness:
            Serial.print(" | P2P_IR:"); Serial.print(P2P_IR);
            Serial.print(" P2P_Red:"); Serial.print(P2P_Red);
            Serial.print(" Duty_IR:"); Serial.print(duty_IR);
            Serial.print(" Duty_Red:"); Serial.print(duty_Red);
            Serial.println();
        } else {
            Serial.println("Signal_Low"); 
            final_BPM = 0;
            final_SpO2 = 0;
            if(duty_IR < 85) duty_IR++;
            if(duty_Red < 85) duty_Red++;
        }
    }
  }
}