#include "driver/rmt.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= PIN DEFINITIONS =================
#define PIN_IR_WIN    22 
#define PIN_IR_PWM    15 
#define PIN_RED_WIN   19 
#define PIN_RED_PWM   14 

#define PIN_ADC_FEEDBACK  34  // DC επίπεδο (AGC)
#define PIN_ADC_IR_SIG    35  // IR AC σήμα
#define PIN_ADC_RED_SIG   36  // RED AC σήμα

// ================= CONFIGURATION =================
#define FSM_TICK_US       500      // 2 kHz FSM (περισσότερος χρόνος για κάθε LED)
#define RMT_CLK_DIV       80    
#define CARRIER_PERIOD    20       // 50 kHz PWM
#define NUM_PULSES        25    

#define CH_IR_WIN   RMT_CHANNEL_0
#define CH_IR_PWM   RMT_CHANNEL_1
#define CH_RED_WIN  RMT_CHANNEL_2
#define CH_RED_PWM  RMT_CHANNEL_3

const float ADC_TO_MV = 3300.0 / 4095.0;

// ================= GLOBAL VARIABLES =================
rmt_item32_t items_window[2];          
rmt_item32_t items_pwm[NUM_PULSES + 1]; 

volatile int fsm_state = 0;
hw_timer_t * timer = NULL;

int duty_IR = 85; 
int duty_Red = 85;

volatile int ir_sig = 0, red_sig = 0;
volatile int ir_fb = 0, red_fb = 0;

volatile bool newData = false;

float final_BPM = 0;
float final_SpO2 = 0;

// ================= FILTER CLASSES =================
class LowPass {
  float alpha, val;
public:
  LowPass(float a) { alpha = a; val = 0; }
  void set(float x) { val = x; }
  float step(float x) {
    val = alpha * x + (1 - alpha) * val;
    return val;
  }
};

class SmartEnvelope {
  float val, attack, release;
public:
  SmartEnvelope(float a, float r) { val = 0; attack = a; release = r; }
  void reset() { val = 0; }
  float step(float x) {
    float abs_x = abs(x);
    if (abs_x > val) val += attack * (abs_x - val);
    else val -= release * (val - abs_x);
    return val * 2.0; // peak-to-peak
  }
};

// ================= FILTER INSTANCES =================
LowPass agc_IR(0.05), agc_Red(0.05);
LowPass dc_center_IR(0.001), dc_center_Red(0.001);
SmartEnvelope env_IR(0.05, 0.0003), env_Red(0.05, 0.0003);

// ================= RMT FUNCTIONS =================
void loadBufferWithDuty(int dutyPercent) {
  if(dutyPercent < 5) dutyPercent = 5;
  if(dutyPercent > 90) dutyPercent = 90;

  int high_us = (CARRIER_PERIOD * dutyPercent)/100;
  int low_us  = CARRIER_PERIOD - high_us;

  for(int i=0; i<NUM_PULSES; i++)
    items_pwm[i] = {{{ (uint16_t)high_us, 1, (uint16_t)low_us, 0 }}};
  items_pwm[NUM_PULSES] = {{{0,0,0,0}}};
  items_window[0] = {{{500,1,0,0}}};
  items_window[1] = {{{0,0,0,0}}};
}

void setupRMT() {
  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX;
  config.clk_div = RMT_CLK_DIV;
  config.mem_block_num = 1;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  config.channel = CH_IR_WIN; config.gpio_num = (gpio_num_t)PIN_IR_WIN; rmt_config(&config); rmt_driver_install(config.channel,0,0);
  config.channel = CH_IR_PWM; config.gpio_num = (gpio_num_t)PIN_IR_PWM; rmt_config(&config); rmt_driver_install(config.channel,0,0);
  config.channel = CH_RED_WIN; config.gpio_num = (gpio_num_t)PIN_RED_WIN; rmt_config(&config); rmt_driver_install(config.channel,0,0);
  config.channel = CH_RED_PWM; config.gpio_num = (gpio_num_t)PIN_RED_PWM; rmt_config(&config); rmt_driver_install(config.channel,0,0);
}

// ================= FSM ISR =================
void IRAM_ATTR onTimerISR() {
  switch(fsm_state) {
    case 0: // IR FIRE
      loadBufferWithDuty(duty_IR);
      rmt_write_items(CH_IR_WIN, items_window, 2, false);
      rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES+1, false);
      break;
    case 1: // IR SAMPLE
      ir_fb  = analogRead(PIN_ADC_FEEDBACK);
      ir_sig = analogRead(PIN_ADC_IR_SIG);
      break;
    case 2: // Pause
      break;
    case 3: // Pause End
      break;
    case 4: // RED FIRE
      loadBufferWithDuty(duty_Red);
      rmt_write_items(CH_RED_WIN, items_window, 2, false);
      rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES+1, false);
      break;
    case 5: // RED SAMPLE
      red_fb  = analogRead(PIN_ADC_FEEDBACK);
      red_sig = analogRead(PIN_ADC_RED_SIG);
      newData = true;
      break;
    case 6: // Pause
      break;
    case 7: // Pause End
      break;
  }
  fsm_state = (fsm_state+1) % 8;
}

// ================= SETUP =================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG,0);
  Serial.begin(115200);
  analogReadResolution(12); analogSetAttenuation(ADC_11db);

  pinMode(PIN_ADC_FEEDBACK, INPUT);
  pinMode(PIN_ADC_IR_SIG, INPUT);
  pinMode(PIN_ADC_RED_SIG, INPUT);

  setupRMT();

  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimerISR);
  timerAlarm(timer, FSM_TICK_US, true, 0);

  Serial.println("Pulse Oximeter Running: IR/RED separated.");
}

// ================= MAIN LOOP =================
void loop() {
  static unsigned long lastBeat = 0;
  static bool beatLatch = false;
  static unsigned long lastSpO2 = 0;

  if(!newData) return;
  newData = false;

  // -------- Convert to mV --------
  float IR_mV  = ir_sig * ADC_TO_MV;
  float RED_mV = red_sig * ADC_TO_MV;
  float FB_IR  = ir_fb * ADC_TO_MV;
  float FB_Red = red_fb * ADC_TO_MV;

  // -------- AGC --------
  float FB_IR_s  = agc_IR.step(FB_IR);
  float FB_Red_s = agc_Red.step(FB_Red);

  if(FB_IR_s > 1600 && duty_IR > 10) duty_IR--;
  if(FB_IR_s < 1400 && duty_IR < 90) duty_IR++;
  if(FB_Red_s > 1600 && duty_Red > 10) duty_Red--;
  if(FB_Red_s < 1400 && duty_Red < 90) duty_Red++;

  // -------- Remove DC --------
  float AC_IR  = IR_mV - dc_center_IR.step(IR_mV);
  float AC_Red = RED_mV - dc_center_Red.step(RED_mV);

  // -------- Envelope --------
  float P2P_IR  = env_IR.step(AC_IR);
  float P2P_Red = env_Red.step(AC_Red);

  // -------- BPM --------
  if(AC_IR > 50 && !beatLatch) {
    beatLatch = true;
    unsigned long now = millis();
    if(now - lastBeat > 300) {
      final_BPM = 60000.0 / (now - lastBeat);
      lastBeat = now;
    }
  }
  if(AC_IR < -50) beatLatch = false;

  // -------- SpO2 --------
  if(millis() - lastSpO2 > 200) {
    lastSpO2 = millis();
    if(P2P_IR > 10) {
      float R = (P2P_Red / FB_Red_s) / (P2P_IR / FB_IR_s);
      float spo2 = 110 - 25 * R;
      if(spo2 > 100) spo2 = 100;
      if(spo2 < 60) spo2 = 60;
      final_SpO2 = final_SpO2*0.9 + spo2*0.1;
    }
  }

  Serial.print("BPM: "); Serial.print(final_BPM);
  Serial.print(" | SpO2: "); Serial.print(final_SpO2);
  Serial.print(" | IR Duty: "); Serial.print(duty_IR);
  Serial.print(" | RED Duty: "); Serial.println(duty_Red);
}