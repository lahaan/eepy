// MX-FS-03V TX via the RMT peripheral, rc-switch protocol 1.
// Timings must match sui77/rc-switch default (see RCSwitch.cpp proto[0]):
//   pulse=350us, sync={1,31}, zero={1,3}, one={3,1}, MSB-first, repeatTx.
// RMT generates the pulses in hardware: a CPU bit-bang got its timing broken
// by BLE/WiFi interrupts (~25% of codes lost once NimBLE was enabled).
// One code = 32 bit symbols + 1 sync = 33 symbols, fits the 48-symbol channel
// memory, so hardware loop mode repeats it with no refill interrupts.
// rc-switch only decodes a code framed by syncs on both sides, so a lone sync
// goes out first; otherwise the first repeat is always wasted.
// Module: VCC->5V, GND->GND, DATA->EEPY_ESP32_TX_PIN (3.3V logic is enough
// to drive the FS03V input). 17.3cm antenna wire on ANT pad.

#include "rf_tx.h"
#include "rf_proto.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

static const char *TAG = "rftx";

// rc-switch protocol 1 constants
#define RF_PULSE_US 350
#define RF_REPEAT 4 // 10 is lib default; 4 x ~56ms = ~224ms on air per code
#define RMT_RES_HZ 1000000 // 1 tick = 1us
#define RMT_MEM_SYMBOLS 48
#define TX_WAIT_MS 500

static rmt_channel_handle_t chan;
static rmt_encoder_handle_t enc;
// Read by the RMT driver during a transmission; only rewritten after the
// previous one has finished.
static rmt_symbol_word_t syms[EEPY_RF_BITLEN + 1];
static rmt_symbol_word_t lead_sync;

static rmt_symbol_word_t pulse(int high_mult, int low_mult) {
  return (rmt_symbol_word_t){
      .duration0 = RF_PULSE_US * high_mult,
      .level0 = 1,
      .duration1 = RF_PULSE_US * low_mult,
      .level1 = 0,
  };
}

void rf_tx_init(void) {
  rmt_tx_channel_config_t cc = {
      .gpio_num = (gpio_num_t)EEPY_ESP32_TX_PIN,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_RES_HZ,
      .mem_block_symbols = RMT_MEM_SYMBOLS,
      .trans_queue_depth = 2, // lead-in sync + looped code
  };
  rmt_copy_encoder_config_t ec = {};
  esp_err_t r = rmt_new_tx_channel(&cc, &chan);
  if (r == ESP_OK) r = rmt_new_copy_encoder(&ec, &enc);
  if (r == ESP_OK) r = rmt_enable(chan);
  ESP_LOGI(TAG, "TX init GPIO%d (RMT) -> MX-FS-03V DATA (%s)", EEPY_ESP32_TX_PIN,
           esp_err_to_name(r));
  if (r != ESP_OK) chan = NULL;
}

// Queues the code and returns; the ~224ms on air runs in hardware.
void rf_tx_send(uint32_t code) {
  if (!chan) return;
  if (rmt_tx_wait_all_done(chan, TX_WAIT_MS) != ESP_OK) {
    ESP_LOGW(TAG, "previous code still sending, dropped");
    return;
  }
  for (int i = 0; i < (int)EEPY_RF_BITLEN; i++) {
    bool one = code & (1UL << (EEPY_RF_BITLEN - 1 - i)); // MSB first
    syms[i] = one ? pulse(3, 1) : pulse(1, 3);
  }
  syms[EEPY_RF_BITLEN] = pulse(1, 31); // sync
  lead_sync = pulse(1, 31);
  rmt_transmit_config_t lead = {.loop_count = 0, .flags.eot_level = 0};
  rmt_transmit_config_t tc = {.loop_count = RF_REPEAT, .flags.eot_level = 0};
  esp_err_t r = rmt_transmit(chan, enc, &lead_sync, sizeof(lead_sync), &lead);
  if (r == ESP_OK) r = rmt_transmit(chan, enc, syms, sizeof(syms), &tc);
  if (r != ESP_OK) ESP_LOGW(TAG, "rmt_transmit: %s", esp_err_to_name(r));
}
