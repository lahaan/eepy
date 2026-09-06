#pragma once
#include <stdint.h>

// Shared ESP32 -> Pico 433MHz protocol.
// FS1000A (TX, 3 pins: VCC GND DATA) + XY-MK-5V (RX, 4 pins: VCC GND DATA DATA)
// are dumb ASK/OOK modules: they only pass pulses, so a packet lib is needed.
//
// NOTE: RadioHead RH_ASK (ideal, arbitrary payloads incl. SSID strings) does
// NOT compile on RP2350 Arduino core yet (TIMER_IRQ_1 removed in Pico SDK 2.x).
// Boilerplate therefore uses rc-switch 32-bit codes (compiles everywhere).
// Full SSID TX needs a RadioHead fix or custom 4b6b ASK layer later.
//
// 32-bit code layout (MSB first), sent with RCSwitch.send(code, 32):
//   [31:24] seq     - scan counter
//   [23:19] index   - AP index (0..31, MAX_APS is 20)
//   [18:14] total   - APs in scan (0..31)
//   [13:7]  rssi100 - (rssi + 100), 0..100 (e.g. -52dBm -> 48)
//   [6:3]   channel - WiFi channel clamped 0..15
//   [2:0]   reserved (0)

#ifdef __cplusplus
extern "C" {
#endif

#define EEPY_RF_BITLEN 32u

// FS1000A (TX) + XY-MK-5V (RX) wiring:
//   ESP32-C3 GPIO4 -> TX module DATA (3.3V logic OK, module VCC to board 5V/VU)
//   RX module DATA -> 10k -> Pico GP16 -> 20k -> GND (=3.3V tap, orientation matters)
//   RX module VCC from Pico VBUS/VSYS (5V when USB plugged), GND to Pico GND.
//   RP2350 GPIOs are NOT 5V-tolerant. Antennas: 17.3cm wire on both ANT pads.
#define EEPY_ESP32_TX_PIN 4
#define EEPY_PICO_RX_PIN 16

static inline uint32_t eepy_encode(uint8_t seq, uint8_t index, uint8_t total,
                                   int rssi_dbm, uint8_t channel) {
  int r100 = rssi_dbm + 100;
  if (r100 < 0) r100 = 0;
  if (r100 > 100) r100 = 100;
  if (index > 31) index = 31;
  if (total > 31) total = 31;
  uint8_t ch = channel > 15 ? 15 : channel;
  return ((uint32_t)seq << 24) | ((uint32_t)(index & 31) << 19) |
         ((uint32_t)(total & 31) << 14) | ((uint32_t)(r100 & 127) << 7) |
         ((uint32_t)(ch & 15) << 3);
}

static inline void eepy_decode(uint32_t code, uint8_t *seq, uint8_t *index,
                               uint8_t *total, int *rssi_dbm, uint8_t *channel) {
  if (seq) *seq = (code >> 24) & 0xFF;
  if (index) *index = (code >> 19) & 0x1F;
  if (total) *total = (code >> 14) & 0x1F;
  if (rssi_dbm) *rssi_dbm = (int)((code >> 7) & 0x7F) - 100;
  if (channel) *channel = (code >> 3) & 0x0F;
}

#ifdef __cplusplus
}
#endif
