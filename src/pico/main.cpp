// Pico 2W 433MHz RX hub (boilerplate, no hardware needed to compile).
// Receives 32-bit rc-switch codes from ESP32 via FS1000A/XY-MK-5V modules.
// Wiring (after soldering): RX DATA -> GP16 via div (10k/20k),
// common GND, 17.3cm antenna wire (optional).
// pio run -e pico2w -t upload && pio device monitor -e pico2w

#include <Arduino.h>
#include <RCSwitch.h>
#include "rf_proto.h"

static RCSwitch rx;
static unsigned long got = 0, ignored = 0;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }
  Serial.println("[pico-hub] boot, waiting for 433MHz packets...");
  rx.setReceiveTolerance(60);
  rx.enableReceive(EEPY_PICO_RX_PIN);
  Serial.println("[pico-hub] rc-switch RX ready on GP16 @115200 USB-serial");
  Serial.println("[pico-hub] tip: `pio device monitor -e pico2w` (no JTAG needed)");
}

void loop() {
  if (rx.available()) {
    unsigned long code = rx.getReceivedValue();
    unsigned int bits = rx.getReceivedBitlength();
    if (code == 0 || bits != EEPY_RF_BITLEN) {
      ignored++;
      Serial.printf("[pico-hub] ignored: value=%lu bits=%u (ignored=%lu)\n", code, bits, ignored);
    } else {
      got++;
      uint8_t seq, idx, total, ch;
      int rssi;
      eepy_decode((uint32_t)code, &seq, &idx, &total, &rssi, &ch);
      Serial.printf("[pico-hub] %u/%u seq=%u rssi=%d ch=%u (raw %lu, got=%lu)\n",
                    idx + 1, total, seq, rssi, ch, code, got);
      // TODO: push to web UI ring buffer / BLE GATT or have CLI to dump to 
    }
    rx.resetAvailable();
  }
}
