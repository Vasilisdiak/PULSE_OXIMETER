#include "driver/rmt.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================
#define PIN_IR_WIN    23    
#define PIN_IR_PWM    15 
#define PIN_RED_WIN   19 
#define PIN_RED_PWM   14 

// HARDWARE CONFIGURATION
#define PIN_ADC_DC    34  
#define PIN_ADC_AC    35  

// =============================================================
// ------------------- TIMING SETTINGS -------------------------
// =============================================================
#define FSM_TICK_US     250
#define RMT_CLK_DIV     80    
#define CARRIER_PERIOD  20    
#define NUM_PULSES      25    

#define CH_IR_WIN   RMT_CHANNEL_0
#define CH_IR_PWM   RMT_CHANNEL_1
#define CH_RED_WIN  RMT_CHANNEL_2
#define CH_RED_PWM  RMT_CHANNEL_3

// =============================================================
// ------------------ LCD SETUP --------------------------------
// =============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =============================================================
// ------------------ GLOBAL VARIABLES -------------------------
// =============================================================
rmt_item32_t items_window[2];          
rmt_item32_t items_pwm[NUM_PULSES + 1]; 

volatile int fsm_state = 0;
hw_timer_t * timer = NULL;
int current_duty_percent = 85; 

volatile int ir_dc_val = 0;
volatile int ir_ac_val = 0;
volatile int red_dc_val = 0;
volatile int red_ac_val = 0;
volatile bool newData = false;

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
    val = alpha * x + (1.0 - alpha) * val;
    return val;
  }
};

LowPass avgDC_IR(0.01);
LowPass avgDC_Red(0.01);
LowPass smoothAC_IR(0.2);
LowPass smoothAC_Red(0.2);
LowPass avgMag_IR(0.02);
LowPass avgMag_Red(0.02);

// =============================================================
// ------------------- RMT FUNCTIONS ---------------------------
// =============================================================
void setDutyCycle(int dutyPercent) {
  dutyPercent = constrain(dutyPercent, 5, 90);
  int high_us = (CARRIER_PERIOD * dutyPercent) / 100;
  int low_us  = CARRIER_PERIOD - high_us;
  if (high_us < 1) high_us = 1;
  if (low_us < 1) low_us = 1;

  for (int i = 0; i < NUM_PULSES; i++)
    items_pwm[i] = {{{ (uint16_t)high_us, 1, (uint16_t)low_us, 0 }}};

  items_pwm[NUM_PULSES] = {{{ 0, 0, 0, 0 }}};
  items_window[0] = {{{ 500, 1, 0, 0 }}};
  items_window[1] = {{{ 0, 0, 0, 0 }}};
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

  config.channel = CH_IR_WIN;  config.gpio_num = (gpio_num_t)PIN_IR_WIN;  rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
  config.channel = CH_IR_PWM;  config.gpio_num = (gpio_num_t)PIN_IR_PWM;  rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
  config.channel = CH_RED_WIN; config.gpio_num = (gpio_num_t)PIN_RED_WIN; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
  config.channel = CH_RED_PWM; config.gpio_num = (gpio_num_t)PIN_RED_PWM; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
}

// =============================================================
// ------------------- TIMER ISR -------------------------------
// =============================================================
void IRAM_ATTR onTimerISR() {
  switch (fsm_state) {
    case 0:
      rmt_write_items(CH_IR_WIN, items_window, 2, false);
      rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES + 1, false);
      break;

    case 1:
      ir_dc_val = analogRead(PIN_ADC_DC);
      ir_ac_val = analogRead(PIN_ADC_AC);
      break;

    case 4:
      rmt_write_items(CH_RED_WIN, items_window, 2, false);
      rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES + 1, false);
      break;

    case 5:
      red_dc_val = analogRead(PIN_ADC_DC);
      red_ac_val = analogRead(PIN_ADC_AC);
      newData = true;
      break;
  }
  fsm_state = (fsm_state + 1) % 8;
}

// =============================================================
// ----------------------- SETUP -------------------------------
// =============================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_ADC_DC, INPUT);
  pinMode(PIN_ADC_AC, INPUT);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Pulse Oximeter");

  setupRMT();
  setDutyCycle(current_duty_percent);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimerISR);
  timerAlarm(timer, FSM_TICK_US, true, 0);
}

// =============================================================
// ----------------------- LOOP --------------------------------
// =============================================================
void loop() {
  static unsigned long last_beat = 0;
  static unsigned long last_spo2 = 0;

  if (newData) {
    newData = false;

    float voltageDC = ir_dc_val * (3.3 / 4095.0);
    if (voltageDC > 1.5 && current_duty_percent > 5) setDutyCycle(--current_duty_percent);
    else if (voltageDC < 1.4 && current_duty_percent < 95) setDutyCycle(++current_duty_percent);

    float dcIR = avgDC_IR.step(ir_dc_val);
    float dcRed = avgDC_Red.step(red_dc_val);

    static float baseIR = 0, baseRed = 0;
    baseIR = 0.95 * baseIR + 0.05 * ir_ac_val;
    baseRed = 0.95 * baseRed + 0.05 * red_ac_val;

    float pulseIR = ir_ac_val - baseIR;
    float pulseRed = red_ac_val - baseRed;

    float cleanIR = smoothAC_IR.step(pulseIR);

    float magIR = avgMag_IR.step(abs(pulseIR));
    float magRed = avgMag_Red.step(abs(pulseRed));

    if (cleanIR > magIR * 1.5 && millis() - last_beat > 300) {
      final_BPM = 60000.0 / (millis() - last_beat);
      last_beat = millis();
    }

    if (millis() - last_spo2 > 500 && voltageDC > 0.5) {
      float R = (magRed / dcRed) / (magIR / dcIR);
      final_SpO2 = constrain(110 - 25 * R, 70, 100);
      last_spo2 = millis();
    }

    lcd.setCursor(0,0);
    lcd.print("BPM: ");
    lcd.print((int)final_BPM);
    lcd.print("   ");

    lcd.setCursor(0,1);
    lcd.print("SpO2: ");
    lcd.print((int)final_SpO2);
    lcd.print("%   ");
  }
}
