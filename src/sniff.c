// Promiscuous beacon sniffer for the locked AP (see sniff.h).

#include <string.h>
#include "sniff.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"

#define FC_BEACON     0x80
#define HDR_LEN       24
#define ADDR2_OFF     10 // transmitter address (== BSSID for AP mgmt frames)

static uint8_t target[6];
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static int32_t sum;
static int count;

// Runs in the WiFi task for every management frame on the channel.
static void rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *f = p->payload;
  if (p->rx_ctrl.sig_len < HDR_LEN) return;
  // Beacons only: probe responses arrive in bursts right after our own scan
  // and would inflate the beacons/s rate.
  if (f[0] != FC_BEACON) return;
  if (memcmp(f + ADDR2_OFF, target, 6) != 0) return;

  int rssi = p->rx_ctrl.rssi;
  portENTER_CRITICAL(&mux);
  sum += rssi;
  count++;
  portEXIT_CRITICAL(&mux);
}

esp_err_t sniff_start(const uint8_t bssid[6], uint8_t channel) {
  memcpy(target, bssid, 6);
  portENTER_CRITICAL(&mux);
  sum = 0;
  count = 0;
  portEXIT_CRITICAL(&mux);

  wifi_promiscuous_filter_t flt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
  esp_wifi_set_promiscuous_filter(&flt);
  esp_wifi_set_promiscuous_rx_cb(rx_cb);
  esp_err_t r = esp_wifi_set_promiscuous(true);
  if (r == ESP_OK) r = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  return r;
}

void sniff_stop(void) {
  esp_wifi_set_promiscuous(false);
}

int sniff_take(int *mean_rssi) {
  portENTER_CRITICAL(&mux);
  int n = count;
  int32_t s = sum;
  sum = 0;
  count = 0;
  portEXIT_CRITICAL(&mux);
  if (n > 0 && mean_rssi) *mean_rssi = (int)(s / n);
  return n;
}
