#include "driver/rmt.h"
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================
#define PIN_IR_WIN    22 //PIR
#define PIN_IR_PWM    15 //CIR 
#define PIN_RED_WIN   19 //PR
#define PIN_RED_PWM   14 //CR

// --- NEW: DUAL ADC SETUP ---
#define PIN_ADC_DC    34 // Connect to s_ph (First OpAmp Output) -> Controls Brightness
#define PIN_ADC_AC    35 // Connect to OA2 Pin 7 (Second OpAmp Output) -> Shows Heartbeat

// =============================================================
// ------------------- TIMING SETTINGS -------------------------
// =============================================================
#define FSM_TICK_US     500   
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

// =============================================================
// --------------- DUTY CYCLE FUNCTION -------------------------
// =============================================================
void setDutyCycle(int dutyPercent) {
    if (dutyPercent < 5) dutyPercent = 5;
    if (dutyPercent > 90) dutyPercent = 90;

    int high_us = (CARRIER_PERIOD * dutyPercent) / 100;
    int low_us  = CARRIER_PERIOD - high_us;
    if (high_us < 1) high_us = 1; 
    if (low_us < 1) low_us = 1;

    for(int i=0; i < NUM_PULSES; i++) {
        items_pwm[i] = {{{ (uint16_t)high_us, 1, (uint16_t)low_us, 0 }}};
    }
    items_pwm[NUM_PULSES] = {{{ 0, 0, 0, 0 }}}; 
    items_window[0] = {{{ 500, 1, 0, 0 }}}; 
    items_window[1] = {{{ 0, 0, 0, 0 }}}; 
}

// =============================================================
// ---------------- SETUP: RMT CONFIG --------------------------
// =============================================================
void setupRMT() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;
    config.tx_config.loop_en = false; 
    config.tx_config.carrier_en = false; 
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    config.channel = CH_IR_WIN;  config.gpio_num = (gpio_num_t)PIN_IR_WIN; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
    config.channel = CH_IR_PWM;  config.gpio_num = (gpio_num_t)PIN_IR_PWM; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
    config.channel = CH_RED_WIN; config.gpio_num = (gpio_num_t)PIN_RED_WIN; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
    config.channel = CH_RED_PWM; config.gpio_num = (gpio_num_t)PIN_RED_PWM; rmt_config(&config); rmt_driver_install(config.channel, 0, 0);
}

// =============================================================
// ------------------- THE FSM INTERRUPT -----------------------
// =============================================================
void IRAM_ATTR onTimerISR() {
    switch (fsm_state) {
        case 0: // IR PHASE
            rmt_write_items(CH_IR_WIN, items_window, 2, false); 
            rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES + 1, false);
            break;
        case 2: // RED PHASE
            rmt_write_items(CH_RED_WIN, items_window, 2, false);
            rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES + 1, false);
            break;
    }
    fsm_state = (fsm_state + 1) % 4;
}

// =============================================================
// ----------------------- MAIN SETUP --------------------------
// =============================================================
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Fix Brownout Crash
    Serial.begin(115200);

    // Setup ADC
    analogReadResolution(12);       
    analogSetAttenuation(ADC_11db); 
    pinMode(PIN_ADC_DC, INPUT);
    pinMode(PIN_ADC_AC, INPUT);

    setupRMT();
    setDutyCycle(current_duty_percent); 

    timer = timerBegin(1000000); 
    timerAttachInterrupt(timer, &onTimerISR);
    timerAlarm(timer, FSM_TICK_US, true, 0); 

    Serial.println("System Running. Plotting Heartbeat...");
}

// =============================================================
// ------------------- MAIN LOOP (AGC + PLOTTING) --------------
// =============================================================
void loop() {
    static unsigned long lastCheck = 0;
    
    // Run loop every 20ms (50Hz plotter update)
    if (millis() - lastCheck > 20) {
        lastCheck = millis();

        // 1. Read BOTH sensors
        int rawDC = analogRead(PIN_ADC_DC); // For AGC
        int rawAC = analogRead(PIN_ADC_AC); // For Heartbeat
        
        float voltageDC = rawDC * (3.3 / 4095.0);
        float voltageAC = rawAC * (3.3 / 4095.0);

        // 2. AGC Logic (Uses DC signal)
        // Aim for 1.8V on the DC line
        bool changed = false;
        if (voltageDC > 1.5) {
            // Temporary limit: Don't go below 50% (was 2%)
            if (current_duty_percent > 5) { current_duty_percent--; changed = true; } 
        } else if (voltageDC < 1.4) {
            // Temporary limit: Don't go above 50% (was 90%)
            if (current_duty_percent < 95) { current_duty_percent++; changed = true; } 
        }
        if (changed) setDutyCycle(current_duty_percent);

        // 3. PLOT THE HEARTBEAT & DUTY CYCLE
        Serial.print("DC_Level:");
        Serial.print(voltageDC);
        Serial.print(" Pulse_Wave:");
        Serial.print(voltageAC); 
        // ADDED: Feedback for Duty Cycle
        Serial.print(" Duty:");
        Serial.println(current_duty_percent);
    }
}