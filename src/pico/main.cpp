// Pico 2W 433MHz RX hub (boilerplate, no hardware needed to compile).
// Receives 32-bit rc-switch codes from ESP32 via FS1000A/XY-MK-5V modules.
// Wiring (after soldering): RX DATA -> GP16 via divider (RX outputs 5V!),
// common GND, 17.3cm antenna wire. Then:
//   pio run -e pico2w -t upload && pio device monitor -e pico2w

#include <Arduino.h>
#include <RCSwitch.h>
#include "rf_proto.h"

static RCSwitch rx;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }
  Serial.println("[pico-hub] boot, waiting for 433MHz packets...");
  rx.enableReceive(EEPY_PICO_RX_PIN);
  Serial.println("[pico-hub] rc-switch RX ready on GP16");
}

void loop() {
  if (rx.available()) {
    unsigned long code = rx.getReceivedValue();
    unsigned int bits = rx.getReceivedBitlength();
    if (code == 0 || bits != EEPY_RF_BITLEN) {
      Serial.printf("[pico-hub] ignored: value=%lu bits=%u\n", code, bits);
    } else {
      uint8_t seq, idx, total, ch;
      int rssi;
      eepy_decode((uint32_t)code, &seq, &idx, &total, &rssi, &ch);
      Serial.printf("[pico-hub] %u/%u seq=%u rssi=%d ch=%u (raw %lu)\n",
                    idx + 1, total, seq, rssi, ch, code);
      // TODO: push to web UI ring buffer / BLE GATT characteristic.
    }
    rx.resetAvailable();
  }
}
