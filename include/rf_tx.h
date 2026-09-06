#pragma once
#include <stdint.h>

// MX-FS-03V ASK transmitter (ESP-IDF, rc-switch protocol 1 compatible).
// No Arduino dependency: bit-bangs EEPY_ESP32_TX_PIN with esp_rom_delay_us.
// Pairs with Pico RCSwitch RX (protocol auto-detect, 32-bit codes).

#ifdef __cplusplus
extern "C" {
#endif

void rf_tx_init(void);
void rf_tx_send(uint32_t code);

#ifdef __cplusplus
}
#endif
