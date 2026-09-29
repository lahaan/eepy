#pragma once
#include <stdint.h>
#include "esp_err.h"

// Locked-AP RSSI sampler. Puts the radio in promiscuous mode on the target's
// channel and keeps beacons sent by the target BSSID.
// APs beacon every ~102ms, so this gives ~10 samples/s without associating.
// Call sniff_stop() before a full scan (scan fails while promiscuous).

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sniff_start(const uint8_t bssid[6], uint8_t channel);
void sniff_stop(void);

// Samples since the last call: returns count, writes mean RSSI (if count > 0).
int sniff_take(int *mean_rssi);

#ifdef __cplusplus
}
#endif
