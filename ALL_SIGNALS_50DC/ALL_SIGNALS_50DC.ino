#include "driver/rmt.h"
#include <Arduino.h>

// =============================================================
// -------------------- PIN DEFINITIONS ------------------------
// =============================================================
#define PIN_PIR     22
#define PIN_CIR     15
#define PIN_PR      19
#define PIN_CR      14

// =============================================================
// ------------------- TIMING SETTINGS -------------------------
// =============================================================
// The "Beat" of our FSM. 500us active, 500us dead.
#define FSM_TICK_US     500 

// Carrier Wave Settings (50kHz)
#define RMT_CLK_DIV     80   // 1us resolution
// NEW: 20us Period = 50kHz
#define CARRIER_PERIOD  20   
// NEW: 25 pulses * 20us = 500us (Fills the window perfectly)
#define NUM_PULSES      25   

// RMT Channels
#define CH_PIR  RMT_CHANNEL_0
#define CH_CIR  RMT_CHANNEL_1
#define CH_PR   RMT_CHANNEL_2 
#define CH_CR   RMT_CHANNEL_3

// =============================================================
// ------------------ MANUAL BUFFERS ---------------------------
// =============================================================
// Buffers for the "Window" (Solid HIGH)
rmt_item32_t manual_window_item[2];

// Buffers for the "Carrier" (PWM Pulses)
rmt_item32_t manual_pwm_items[NUM_PULSES + 1];

// FSM State Variable
volatile int fsm_state = 0;
hw_timer_t * timer = NULL;

// =============================================================
// ---------------- SETUP: MANUAL BUFFER FILL ------------------
// =============================================================
void fillManualBuffers() {
    // 1. Define the "Window" Signal (Solid HIGH for 500us)
    manual_window_item[0] = {{{ 500, 1, 0, 0 }}}; 
    manual_window_item[1] = {{{ 0, 0, 0, 0 }}}; // End Marker

    // 2. Define the "PWM" Signal (25 pulses of 50kHz)
    // 50kHz = 20us Period -> 10us High, 10us Low
    int high_t = 10; 
    int low_t  = 10; 

    for(int i=0; i<NUM_PULSES; i++) {
        // Manually constructing the pulse: 10us High, 10us Low
        manual_pwm_items[i] = {{{ (uint16_t)high_t, 1, (uint16_t)low_t, 0 }}};
    }
    // End Marker
    manual_pwm_items[NUM_PULSES] = {{{ 0, 0, 0, 0 }}}; 
}

// =============================================================
// ---------------- SETUP: RMT DRIVER --------------------------
// =============================================================
void setupRMT() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;
    
    // CRITICAL: We do NOT loop in hardware. We fire once when told.
    config.tx_config.loop_en = false; 
    config.tx_config.carrier_en = false; 
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    // Configure all 4 channels
    config.channel = CH_PIR; config.gpio_num = (gpio_num_t)PIN_PIR; rmt_config(&config); rmt_driver_install(CH_PIR, 0, 0);
    config.channel = CH_CIR; config.gpio_num = (gpio_num_t)PIN_CIR; rmt_config(&config); rmt_driver_install(CH_CIR, 0, 0);
    config.channel = CH_PR;  config.gpio_num = (gpio_num_t)PIN_PR;  rmt_config(&config); rmt_driver_install(CH_PR, 0, 0);
    config.channel = CH_CR;  config.gpio_num = (gpio_num_t)PIN_CR;  rmt_config(&config); rmt_driver_install(CH_CR, 0, 0);
}

// =============================================================
// ------------------- THE FSM INTERRUPT -----------------------
// =============================================================
void IRAM_ATTR onTimerISR() {
    // This ISR runs exactly every 500us.
    
    switch (fsm_state) {
        case 0: // === IR PHASE ===
            // Fire IR Window and IR PWM immediately
            rmt_write_items(CH_PIR, manual_window_item, 2, false); 
            rmt_write_items(CH_CIR, manual_pwm_items, NUM_PULSES+1, false);
            break;

        case 1: // === DEAD PHASE (Silence) ===
            // Do nothing. 
            break;

        case 2: // === RED PHASE ===
            // Fire Red Window and Red PWM immediately
            rmt_write_items(CH_PR, manual_window_item, 2, false);
            rmt_write_items(CH_CR, manual_pwm_items, NUM_PULSES+1, false);
            break;

        case 3: // === DEAD PHASE (Silence) ===
            // Do nothing.
            break;
    }

    // Cycle states: 0 -> 1 -> 2 -> 3 -> 0 ...
    fsm_state = (fsm_state + 1) % 4;
}

// =============================================================
// ----------------------- MAIN SETUP --------------------------
// =============================================================
void setup() {
    Serial.begin(115200);
    
    fillManualBuffers();
    setupRMT();

    // Configure Hardware Timer (1MHz frequency -> 1 tick = 1us)
    timer = timerBegin(1000000); 
    timerAttachInterrupt(timer, &onTimerISR);
    timerAlarm(timer, FSM_TICK_US, true, 0); 

    Serial.println("FSM System Started @ 50kHz.");
}

void loop() {
    delay(1000);
}