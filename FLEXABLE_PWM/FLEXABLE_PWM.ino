#include "driver/rmt.h"
#include <Arduino.h>

// ---------------- PIN ASSIGNMENTS ----------------
#define PIN_PIR 5
#define PIN_CIR 15
#define PIN_PR 19
#define PIN_CR 14

// ------------ CONSTANTS FOR ALL CHANNELS ------------
const int PERIOD_US = 2000;       // 2 ms frame
const int DUTY_US   = 500;        // 500 us high for PIR/PR
const int CARRIER_PERIOD_US = 20; // 20 kHz = 50us
const int NUM_PULSES = 25;
const int START_DELAY_US = 1000;
const int END_DELAY_US   = 500;

// Fixed 50% duty cycle
const int high_us = CARRIER_PERIOD_US / 2; // 25 µs
const int low_us  = CARRIER_PERIOD_US / 2; // 25 µs

// ---------------- BUFFERS ----------------
const int MAX_CYCLES = DUTY_US / CARRIER_PERIOD_US;
rmt_item32_t pir_item[1];
rmt_item32_t cir_items[MAX_CYCLES + 1];

rmt_item32_t pr_items[2];
rmt_item32_t cr_items[NUM_PULSES + 1];

void setup() {
    Serial.begin(115200);
    delay(300);

    // ---------------- PIR ----------------
    rmt_config_t cfg_pir = {};
    cfg_pir.channel = RMT_CHANNEL_0;
    cfg_pir.gpio_num = (gpio_num_t)PIN_PIR;
    cfg_pir.mem_block_num = 1;
    cfg_pir.clk_div = 80;
    cfg_pir.tx_config.loop_en = false;
    cfg_pir.tx_config.idle_output_en = true;
    cfg_pir.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pir.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pir);
    rmt_driver_install(cfg_pir.channel, 0, 0);

    pir_item[0].level0 = 1;
    pir_item[0].duration0 = DUTY_US;             // 500us high
    pir_item[0].level1 = 0;
    pir_item[0].duration1 = PERIOD_US - DUTY_US; // 1500us low

    // ---------------- CIR ----------------
    rmt_config_t cfg_cir = {};
    cfg_cir.channel = RMT_CHANNEL_1;
    cfg_cir.gpio_num = (gpio_num_t)PIN_CIR;
    cfg_cir.mem_block_num = 2;
    cfg_cir.clk_div = 80;
    cfg_cir.tx_config.loop_en = false;
    cfg_cir.tx_config.idle_output_en = true;
    cfg_cir.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_cir.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_cir);
    rmt_driver_install(cfg_cir.channel, 0, 0);

    // CIR pulses μέσα στο high του PIR
    for (int i = 0; i < MAX_CYCLES; i++) {
        cir_items[i].level0 = 1;
        cir_items[i].duration0 = high_us;
        cir_items[i].level1 = 0;
        cir_items[i].duration1 = low_us;
    }
    cir_items[MAX_CYCLES].level0 = 0;                  // trailing low για 2ms frame
    cir_items[MAX_CYCLES].duration0 = PERIOD_US - DUTY_US;
    cir_items[MAX_CYCLES].level1 = 0;
    cir_items[MAX_CYCLES].duration1 = 0;

    // ---------------- PR ----------------
    rmt_config_t cfg_pr = {};
    cfg_pr.channel = RMT_CHANNEL_2;
    cfg_pr.gpio_num = (gpio_num_t)PIN_PR;
    cfg_pr.mem_block_num = 1;
    cfg_pr.clk_div = 80;
    cfg_pr.tx_config.loop_en = false;
    cfg_pr.tx_config.idle_output_en = true;
    cfg_pr.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pr.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pr);
    rmt_driver_install(cfg_pr.channel, 0, 0);

    pr_items[0].level0 = 0;
    pr_items[0].duration0 = START_DELAY_US; // 1ms low
    pr_items[0].level1 = 1;
    pr_items[0].duration1 = 500;            // 500us high

    pr_items[1].level0 = 0;
    pr_items[1].duration0 = END_DELAY_US;   // 500us low
    pr_items[1].level1 = 0;
    pr_items[1].duration1 = 0;

    // ---------------- CR ----------------
    rmt_config_t cfg_cr = {};
    cfg_cr.channel = RMT_CHANNEL_3;
    cfg_cr.gpio_num = (gpio_num_t)PIN_CR;
    cfg_cr.mem_block_num = 2;
    cfg_cr.clk_div = 80;
    cfg_cr.tx_config.loop_en = false;
    cfg_cr.tx_config.idle_output_en = true;
    cfg_cr.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_cr.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_cr);
    rmt_driver_install(cfg_cr.channel, 0, 0);

    // CR pulses μέσα στο high του PR
    for (int i = 0; i < NUM_PULSES; i++) {
        cr_items[i].level0 = 1;            // high αμέσως
        cr_items[i].duration0 = high_us;
        cr_items[i].level1 = 0;            // low
        cr_items[i].duration1 = low_us;
    }
    cr_items[NUM_PULSES].level0 = 0;        // trailing low για frame
    cr_items[NUM_PULSES].duration0 = END_DELAY_US;
    cr_items[NUM_PULSES].level1 = 0;
    cr_items[NUM_PULSES].duration1 = 0;
}

void loop() {
    // ----------- STOP ALL CHANNELS ------------
    rmt_tx_stop(RMT_CHANNEL_0);
    rmt_tx_stop(RMT_CHANNEL_1);
    rmt_tx_stop(RMT_CHANNEL_2);
    rmt_tx_stop(RMT_CHANNEL_3);

    // ----------- WRITE ITEMS ------------
    rmt_write_items(RMT_CHANNEL_0, pir_item, 1, false);
    rmt_write_items(RMT_CHANNEL_1, cir_items, MAX_CYCLES + 1, false);
    rmt_write_items(RMT_CHANNEL_2, pr_items, 2, false);
    rmt_write_items(RMT_CHANNEL_3, cr_items, NUM_PULSES + 1, false);

    rmt_wait_tx_done(RMT_CHANNEL_0, portMAX_DELAY);
    rmt_wait_tx_done(RMT_CHANNEL_1, portMAX_DELAY);
    rmt_wait_tx_done(RMT_CHANNEL_2, portMAX_DELAY);
    rmt_wait_tx_done(RMT_CHANNEL_3, portMAX_DELAY);

    // ----------- START ALL CHANNELS TOGETHER ------------
    rmt_tx_start(RMT_CHANNEL_0, true);
    rmt_tx_start(RMT_CHANNEL_1, true);
    rmt_tx_start(RMT_CHANNEL_2, true);
    rmt_tx_start(RMT_CHANNEL_3, true);

    ets_delay_us(PERIOD_US);
}
