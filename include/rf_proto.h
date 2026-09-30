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
// 32-bit code layout (MSB first), sent with RCSwitch.send(code, 32).
// Sample (flags & EEPY_FLAG_CHUNK == 0):
//   [31:29] seq     - packet counter (wraps; only breaks up repeats)
//   [28:24] index   - AP index (0..31, MAX_APS is 20)
//   [23:19] total   - APs in scan (0..31)
//   [18:12] rssi100 - (rssi + 100), 0..100 (e.g. -52dBm -> 48)
//   [11:8]  channel - WiFi channel clamped 0..15
//   [7:3]   tag     - short id of the locked AP's SSID (see below)
//   [2:0]   flags   - EEPY_FLAG_*
// SSID chunk (flags == EEPY_FLAG_CHUNK), two SSID bytes per code:
//   [31:28] chunk   - chunk number 0..15 (bytes 2n, 2n+1; SSIDs are <= 32)
//   [27:20] byte 2n
//   [19:12] byte 2n+1
//   [11:7]  tag     - which SSID this chunk belongs to
//   [6:3]   check   - eepy_chunk_check() over bits [31:7]
//   [2:0]   flags   - EEPY_FLAG_CHUNK
// A 0 byte ends the name (a 32-byte SSID has none). Empty = hidden SSID.
//
// SSID sync: every sample carries the tag, so the Pico only needs the full
// name once per lock; after that the tag confirms its cached copy. 433 is
// one-way (no resend requests), so the ESP32 sends the name on lock, repeats
// it once, then refreshes it every few minutes for a Pico that rebooted.
// Chunks take every other RF slot while pending, so airtime stays at one
// code per slot. The ESP32 never reuses a recent tag for a different AP.

#ifdef __cplusplus
extern "C" {
#endif

#define EEPY_RF_BITLEN 32u

// Live beacon-RSSI sample of the locked AP (vs. a plain scan-list entry).
#define EEPY_FLAG_LOCK 0x1u
// SSID chunk frame (different layout, see above).
#define EEPY_FLAG_CHUNK 0x4u
#define EEPY_TAG_MASK 31u
#define EEPY_SSID_MAX 32u
#define EEPY_SSID_CHUNKS (EEPY_SSID_MAX / 2u)

// FS1000A (TX) + XY-MK-5V (RX) wiring:
//   ESP32-C3 GPIO4 -> TX module DATA (3.3V logic OK, module VCC to board 5V/VU)
//   RX module DATA -> 5.1k -> Pico GP12 (phys pin 16) -> 10k -> GND (=3.3V tap, orientation matters)
//   RX module VCC from Pico VBUS/VSYS (5V when USB plugged), GND to Pico GND.
//   Keep the divider low-ohm: 10k/20k gets stuck high (RP2350-E9 input latch).
//   RP2350 GPIOs are NOT 5V-tolerant. Antennas: 17.3cm wire on both ANT pads.
#define EEPY_ESP32_TX_PIN 4
#define EEPY_PICO_RX_PIN 12

static inline uint32_t eepy_encode(uint8_t seq, uint8_t index, uint8_t total,
                                   int rssi_dbm, uint8_t channel, uint8_t tag,
                                   uint8_t flags) {
  int r100 = rssi_dbm + 100;
  if (r100 < 0) r100 = 0;
  if (r100 > 100) r100 = 100;
  if (index > 31) index = 31;
  if (total > 31) total = 31;
  uint8_t ch = channel > 15 ? 15 : channel;
  return ((uint32_t)(seq & 7) << 29) | ((uint32_t)(index & 31) << 24) |
         ((uint32_t)(total & 31) << 19) | ((uint32_t)(r100 & 127) << 12) |
         ((uint32_t)(ch & 15) << 8) | ((uint32_t)(tag & EEPY_TAG_MASK) << 3) |
         (uint32_t)(flags & 3);
}

static inline void eepy_decode(uint32_t code, uint8_t *seq, uint8_t *index,
                               uint8_t *total, int *rssi_dbm, uint8_t *channel,
                               uint8_t *tag, uint8_t *flags) {
  if (seq) *seq = (code >> 29) & 0x07;
  if (index) *index = (code >> 24) & 0x1F;
  if (total) *total = (code >> 19) & 0x1F;
  if (rssi_dbm) *rssi_dbm = (int)((code >> 12) & 0x7F) - 100;
  if (channel) *channel = (code >> 8) & 0x0F;
  if (tag) *tag = (code >> 3) & EEPY_TAG_MASK;
  if (flags) *flags = code & 0x07;
}

// 4-bit XOR of the nibbles of bits [31:7]; catches most garbled chunks
// (samples carry none: a bad RSSI heals on the next one, a bad name sticks).
static inline uint8_t eepy_chunk_check(uint32_t code) {
  uint32_t v = code >> 7;
  uint8_t x = 0;
  while (v) {
    x ^= v & 0xF;
    v >>= 4;
  }
  return x;
}

static inline uint32_t eepy_encode_chunk(uint8_t chunk, uint8_t b0, uint8_t b1,
                                         uint8_t tag) {
  uint32_t code = ((uint32_t)(chunk & 15) << 28) | ((uint32_t)b0 << 20) |
                  ((uint32_t)b1 << 12) | ((uint32_t)(tag & EEPY_TAG_MASK) << 7) |
                  EEPY_FLAG_CHUNK;
  return code | ((uint32_t)eepy_chunk_check(code) << 3);
}

// Returns 0 if the check nibble doesn't match.
static inline int eepy_decode_chunk(uint32_t code, uint8_t *chunk, uint8_t *b0,
                                    uint8_t *b1, uint8_t *tag) {
  if (((code >> 3) & 0xF) != eepy_chunk_check(code)) return 0;
  if (chunk) *chunk = (code >> 28) & 0xF;
  if (b0) *b0 = (code >> 20) & 0xFF;
  if (b1) *b1 = (code >> 12) & 0xFF;
  if (tag) *tag = (code >> 7) & EEPY_TAG_MASK;
  return 1;
}

#ifdef __cplusplus
}
#endif
