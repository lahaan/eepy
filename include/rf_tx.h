#pragma once
#include <stdint.h>

// MX-FS-03V ASK transmitter (ESP-IDF RMT, rc-switch protocol 1 compatible).
// No Arduino dependency; rf_tx_send() queues the code and returns.
// Pairs with Pico RCSwitch RX (protocol auto-detect, 32-bit codes).

#ifdef __cplusplus
extern "C" {
#endif

void rf_tx_init(void);
void rf_tx_send(uint32_t code);

#ifdef __cplusplus
}
#endif
