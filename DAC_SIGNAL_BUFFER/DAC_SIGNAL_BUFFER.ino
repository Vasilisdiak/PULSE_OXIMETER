#include "driver/rmt.h"
#include "driver/dac.h"
#include <Arduino.h>

// ---------------- PIN ASSIGNMENTS ----------------
#define PIN_PIR 5
#define PIN_PR  19
#define DAC_CH  DAC_CHANNEL_1   // GPIO25

// ---------------- FRAME TIMING ----------------
#define PERIOD_US      2000   // 2ms → 500Hz
#define DUTY_PIR_US     500   // PIR HIGH 0–500us
#define START_PR_US    1000   // PR starts at 1ms
#define DUTY_PR_US      500   // PR HIGH 1000–1500us

// ---------------- SAMPLING ----------------
#define SAMPLE_US        10   // 10us per sample
#define SAMPLES_PER_FRAME (PERIOD_US / SAMPLE_US) // 200

// ---------------- BUFFERS ----------------
uint8_t pir_buffer[SAMPLES_PER_FRAME]; //Κάνει έναν πίνακα με 200 δείγματα
uint8_t pr_buffer[SAMPLES_PER_FRAME];

// ---------------- RMT ----------------
rmt_item32_t pir_item[1];
rmt_item32_t pr_item[1];

void setup() {
    Serial.begin(115200);
    delay(300);

    // ---------------- DAC ----------------
    dac_output_enable(DAC_CH);

    // ---------------- PIR RMT ----------------
    rmt_config_t cfg_pir = {};
    cfg_pir.channel = RMT_CHANNEL_0;
    cfg_pir.gpio_num = (gpio_num_t)PIN_PIR;
    cfg_pir.mem_block_num = 1;
    cfg_pir.clk_div = 80; // 1us tick
    cfg_pir.tx_config.loop_en = true;
    cfg_pir.tx_config.idle_output_en = true;
    cfg_pir.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    cfg_pir.rmt_mode = RMT_MODE_TX;
    rmt_config(&cfg_pir);
    rmt_driver_install(cfg_pir.channel, 0, 0);

    pir_item[0].level0 = 1;
    pir_item[0].duration0 = DUTY_PIR_US;
    pir_item[0].level1 = 0;
    pir_item[0].duration1 = PERIOD_US - DUTY_PIR_US;
    rmt_write_items(RMT_CHANNEL_0, pir_item, 1, true);

    // ---------------- PR RMT ----------------
    rmt_config_t cfg_pr = cfg_pir;
    cfg_pr.channel = RMT_CHANNEL_2;
    cfg_pr.gpio_num = (gpio_num_t)PIN_PR;
    rmt_config(&cfg_pr);
    rmt_driver_install(cfg_pr.channel, 0, 0);

    pr_item[0].level0 = 0;
    pr_item[0].duration0 = START_PR_US;
    pr_item[0].level1 = 1;
    pr_item[0].duration1 = DUTY_PR_US;
    rmt_write_items(RMT_CHANNEL_2, pr_item, 1, true);
}

// =======================================================
// ===================== MAIN LOOP ========================
// =======================================================
void loop() {


    for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
        uint32_t t = i * SAMPLE_US; //Υπολογίζεται ο χρόνος t μέσα στο frame

        // ---------- PIR ----------
        pir_buffer[i] = (t < DUTY_PIR_US) ? 255 : 0;΄// Αν ο χρόνος είναι από τα 0 έως τα 0,5s τότε γράψε 255 (λογικό 1)

        // ---------- PR ----------
        pr_buffer[i] =
            (t >= START_PR_US && t < START_PR_US + DUTY_PR_US) ? 255 : 0;

        // ---------- DAC ----------
        uint8_t dac_value; //Αυτή η μεταβλήτη κρατάει την τιμή DAC

        if (pir_buffer[i] && !pr_buffer[i])
            dac_value = 120;   // Αν η PIR ειναι HIGH, εμφανίζει επίπεδο PIR 
        else if (!pir_buffer[i] && pr_buffer[i])
            dac_value = 200;   // Αν μόνο PR είναι HIGH → DAC εμφανίζει επίπεδο για PR
        else
            dac_value = 0; // Αν κανένα ή ταυτόχρονα και τα δύο δεν είναι HIGH → DAC LOW

        dac_output_voltage(DAC_CH, dac_value); // Στέλνει το δείγμα στο GPIO25

        delayMicroseconds(SAMPLE_US); // Περιμένουμε το χρόνο μεταξύ των δειγμάτων
    }
}

