#include "driver/rmt.h"
#include "driver/dac.h"
#include <Arduino.h>

// ---------------- PIN ASSIGNMENTS ----------------
#define PIN_PIR 5
#define PIN_PR 19
#define DAC_CH DAC_CHANNEL_1   // GPIO25

// ------------ CONSTANTS ------------
const int PERIOD_US      = 2000; // 2ms frame → 500 Hz
const int DUTY_PIR_US    = 500;  // 500us HIGH
const int START_PR_US    = 1000; // PR starts after 1ms
const int DUTY_PR_US     = 500;  // 500us HIGH

// ---------------- RMT ----------------
rmt_item32_t pir_item[1];
rmt_item32_t pr_item[2];

void setup() {
    Serial.begin(115200);
    delay(300);

    // ----------------- DAC -----------------
    dac_output_enable(DAC_CH);

    // ----------------- PIR RMT -----------------
    rmt_config_t cfg_pir = {};
    cfg_pir.channel = RMT_CHANNEL_0;
    cfg_pir.gpio_num = (gpio_num_t)PIN_PIR;
    cfg_pir.mem_block_num = 1;
    cfg_pir.clk_div = 80;
    cfg_pir.tx_config.loop_en = true;  // continuous
    cfg_pir.tx_config.idle_output_en = true;
    cfg_pir.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pir.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pir);
    rmt_driver_install(cfg_pir.channel, 0, 0);

    pir_item[0].level0 = 1;
    pir_item[0].duration0 = DUTY_PIR_US;
    pir_item[0].level1 = 0;
    pir_item[0].duration1 = PERIOD_US - DUTY_PIR_US;
    rmt_write_items(RMT_CHANNEL_0, pir_item, 1, true); // loop

    // ----------------- PR RMT -----------------
    rmt_config_t cfg_pr = {};
    cfg_pr.channel = RMT_CHANNEL_2;
    cfg_pr.gpio_num = (gpio_num_t)PIN_PR;
    cfg_pr.mem_block_num = 1;
    cfg_pr.clk_div = 80;
    cfg_pr.tx_config.loop_en = true;  // continuous
    cfg_pr.tx_config.idle_output_en = true;
    cfg_pr.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pr.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pr);
    rmt_driver_install(cfg_pr.channel, 0, 0);

    pr_item[0].level0 = 0;
    pr_item[0].duration0 = START_PR_US; // 1ms delay
    pr_item[0].level1 = 1;
    pr_item[0].duration1 = DUTY_PR_US;  // 500us HIGH
    rmt_write_items(RMT_CHANNEL_2, pr_item, 1, true); // loop
}

void loop() {
    // Τρέχουσα θέση μέσα στο 2ms frame
    uint32_t t = micros() % PERIOD_US;

    // PIR HIGH 0–500us
    int pir = (t < DUTY_PIR_US) ? 1 : 0;

    // PR HIGH 1000–1500us
    int pr = (t >= START_PR_US && t < START_PR_US + DUTY_PR_US) ? 1 : 0;

    // DAC output
    // 0 = LOW, 128 = PIR, 192 = PR, 255 = both HIGH (ασφαλεία)
    int dac_value = 0;
    if (pir && !pr) dac_value = 128;
    else if (!pir && pr) dac_value = 192;
    else if (pir && pr) dac_value = 255;
    else dac_value = 0;

    dac_output_voltage(DAC_CH, dac_value);

    // μικρή καθυστέρηση για σταθερό 500Hz frame
    delayMicroseconds(10);
}
