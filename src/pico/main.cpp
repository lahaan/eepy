// Pico 2W 433MHz RX hub (boilerplate, no hardware needed to compile).
// Receives 32-bit rc-switch codes from ESP32 via FS1000A/XY-MK-5V modules.
// Wiring (after soldering): RX DATA -> GP12 (phys pin 16) via div (5.1k/10k),
// common GND, 17.3cm antenna wire (optional).
// pio run -e pico2w -t upload && pio device monitor -e pico2w

#include <Arduino.h>
#include <RCSwitch.h>
#include "rf_proto.h"

static RCSwitch rx;
static unsigned long got = 0, ignored = 0;
static unsigned long last_code = 0, last_code_ms = 0;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }
  Serial.println("[pico-hub] boot, waiting for 433MHz packets...");
  // RP2350 pads start isolated and arduino-pico digitalRead() returns 0 without
  // a pinMode; rc-switch's attachInterrupt does neither. INPUT = no pulls
  // (internal pull-down makes the RP2350-E9 latch worse).
  pinMode(EEPY_PICO_RX_PIN, INPUT);
  rx.setReceiveTolerance(60);
  rx.enableReceive(EEPY_PICO_RX_PIN);
  Serial.printf("[pico-hub] rc-switch RX ready on GP%d @115200 USB-serial\n", EEPY_PICO_RX_PIN);
  Serial.println("[pico-hub] tip: `pio device monitor -e pico2w` (no JTAG needed)");
}

void loop() {
  if (rx.available()) {
    unsigned long code = rx.getReceivedValue();
    unsigned int bits = rx.getReceivedBitlength();
    if (code == 0 || bits != EEPY_RF_BITLEN) {
      ignored++;
      Serial.printf("[pico-hub] ignored: value=%lu bits=%u (ignored=%lu)\n", code, bits, ignored);
    } else if (code == last_code && millis() - last_code_ms < 1000) {
      // ESP32 repeats each code RF_REPEAT times; drop the extra decodes.
      last_code_ms = millis();
    } else {
      last_code = code;
      last_code_ms = millis();
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

  // Diag: count raw level changes on the RX pin, report every 10s.
  // ~0 toggles => nothing reaching the pin (wiring/power/divider; stuck
  // level=1 => RP2350-E9 latch, divider bottom leg must be <=~8.2k).
  // Thousands but no packets => signal present, decode problem.
  static int last_lvl = -1;
  static unsigned long toggles = 0, last_report = 0;
  int lvl = digitalRead(EEPY_PICO_RX_PIN);
  if (lvl != last_lvl) {
    toggles++;
    last_lvl = lvl;
  }
  if (millis() - last_report >= 10000) {
    last_report = millis();
    Serial.printf("[pico-hub] alive: GP%d toggles/10s=%lu level=%d got=%lu ignored=%lu\n",
                  EEPY_PICO_RX_PIN, toggles, lvl, got, ignored);
    toggles = 0;
  }
}
