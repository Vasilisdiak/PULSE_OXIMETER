#include "driver/rmt.h"
#include <Arduino.h>

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================
#define PIN_IR_WIN    22 
#define PIN_IR_PWM    15
#define PIN_RED_WIN   19
#define PIN_RED_PWM   14

// =============================================================
// ------------------- TIMING SETTINGS -------------------------
// =============================================================
#define FSM_TICK_US     500   // 500us Window
#define RMT_CLK_DIV     80    // 1us resolution

// FIXED FREQUENCY SETTINGS (50kHz)
#define CARRIER_PERIOD  20    // 20us = 50kHz
#define NUM_PULSES      25    // 25 * 20us = 500us (Fills the window)

// Channels
#define CH_IR_WIN   RMT_CHANNEL_0
#define CH_IR_PWM   RMT_CHANNEL_1
#define CH_RED_WIN  RMT_CHANNEL_2
#define CH_RED_PWM  RMT_CHANNEL_3

// =============================================================
// ------------------ GLOBAL BUFFERS ---------------------------
// =============================================================
rmt_item32_t items_window[2];          
rmt_item32_t items_pwm[NUM_PULSES + 1]; 

// FSM State
volatile int fsm_state = 0;
hw_timer_t * timer = NULL;

// =============================================================
// --------------- DUTY CYCLE FUNCTION -------------------------
// =============================================================

/**
 * @brief Sets the brightness (PWM Duty Cycle).
 * @param dutyPercent Value from 0 to 100.
 */
void setDutyCycle(int dutyPercent) {
    // 1. Clamp input to safe range (5% to 95% to avoid signal loss)
    if (dutyPercent < 5) dutyPercent = 5;
    if (dutyPercent > 95) dutyPercent = 95;

    // 2. Calculate ON and OFF times
    // For 50kHz (20us period):
    // 50% -> 10us High, 10us Low
    // 20% -> 4us High, 16us Low
    int high_us = (CARRIER_PERIOD * dutyPercent) / 100;
    int low_us  = CARRIER_PERIOD - high_us;

    // 3. Update the PWM Buffer
    // We rewrite the "shape" of all 25 pulses in the array
    for(int i=0; i < NUM_PULSES; i++) {
        items_pwm[i] = {{{ (uint16_t)high_us, 1, (uint16_t)low_us, 0 }}};
    }
    // Ensure End Marker is present
    items_pwm[NUM_PULSES] = {{{ 0, 0, 0, 0 }}}; 
    
    // 4. Update the Window Buffer (This stays constant, but good to init here)
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

    // Configure all 4 channels
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
        case 0: // === IR PHASE ===
            rmt_write_items(CH_IR_WIN, items_window, 2, false); 
            rmt_write_items(CH_IR_PWM, items_pwm, NUM_PULSES + 1, false);
            break;

        case 1: // === WAIT ===
            break;

        case 2: // === RED PHASE ===
            rmt_write_items(CH_RED_WIN, items_window, 2, false);
            rmt_write_items(CH_RED_PWM, items_pwm, NUM_PULSES + 1, false);
            break;

        case 3: // === WAIT ===
            break;
    }
    fsm_state = (fsm_state + 1) % 4;
}

// =============================================================
// ----------------------- MAIN SETUP --------------------------
// =============================================================
void setup() {
    Serial.begin(115200);
    
    // Initialize RMT
    setupRMT();
    
    // --- SET INITIAL DUTY CYCLE HERE ---
    setDutyCycle(20); // Example: Start at 20% Brightness

    // Start Timer
    timer = timerBegin(1000000); 
    timerAttachInterrupt(timer, &onTimerISR);
    timerAlarm(timer, FSM_TICK_US, true, 0); 

    Serial.println("System Running. Duty Cycle Variable Ready.");
}

void loop() {
    // Example: Manual Change
    // You can just call setDutyCycle(x) whenever you need to update it.
    delay(2000);
    setDutyCycle(50); // Jump to 50%
    delay(2000);
    setDutyCycle(10); // Drop to 10%
    
    delay(100);
}