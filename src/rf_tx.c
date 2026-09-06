// MX-FS-03V TX via ESP-IDF GPIO bit-bang, rc-switch protocol 1.
// Timings must match sui77/rc-switch default (see RCSwitch.cpp proto[0]):
//   pulse=350us, sync={1,31}, zero={1,3}, one={3,1}, MSB-first, repeatTx.
// Module: VCC->5V, GND->GND, DATA->EEPY_ESP32_TX_PIN (3.3V logic is enough
// to drive the FS03V input). 17.3cm antenna wire on ANT pad.

#include "rf_tx.h"
#include "rf_proto.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "rftx";

// rc-switch protocol 1 constants
#define RF_PULSE_US 350
#define RF_REPEAT 5 // 10 is lib default; 5 keeps 20-AP cycle reasonable (~280ms/AP)

static inline void tx_pulse(int high_mult, int low_mult) {
  gpio_set_level((gpio_num_t)EEPY_ESP32_TX_PIN, 1);
  esp_rom_delay_us((uint32_t)(RF_PULSE_US * high_mult));
  gpio_set_level((gpio_num_t)EEPY_ESP32_TX_PIN, 0);
  esp_rom_delay_us((uint32_t)(RF_PULSE_US * low_mult));
}

void rf_tx_init(void) {
  gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << EEPY_ESP32_TX_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t r = gpio_config(&cfg);
  gpio_set_level((gpio_num_t)EEPY_ESP32_TX_PIN, 0);
  ESP_LOGI(TAG, "TX init GPIO%d -> MX-FS-03V DATA (%s)", EEPY_ESP32_TX_PIN,
           r == ESP_OK ? "ok" : "gpio_config failed");
}

void rf_tx_send(uint32_t code) {
  for (int rep = 0; rep < RF_REPEAT; rep++) {
    for (int i = EEPY_RF_BITLEN - 1; i >= 0; i--) {
      if (code & (1UL << i)) {
        tx_pulse(3, 1); // one
      } else {
        tx_pulse(1, 3); // zero
      }
    }
    tx_pulse(1, 31); // sync
  }
  gpio_set_level((gpio_num_t)EEPY_ESP32_TX_PIN, 0);
}
