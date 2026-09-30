// Pico 2W 433MHz RX hub + BLE bridge.
// Receives 32-bit rc-switch codes from ESP32 via FS1000A/XY-MK-5V modules and
// relays them to the phone page as BLE "eepy-hub" (same NUS UUIDs and JSON
// lines as the ESP32's "eepy-probe"). Read-only: 433 is one-way, so commands
// can't reach the ESP32 through the hub - connect to eepy-probe for control.
// Wiring (after soldering): RX DATA -> GP12 (phys pin 16) via div (5.1k/10k),
// common GND, 17.3cm antenna wire (optional).
// pio run -e pico2w -t upload && pio device monitor -e pico2w
// Output: one JSON object per line on USB serial and BLE (lines not starting
// with '{' are chatter, USB only).

#include <Arduino.h>
#include <BLE.h>
#include <BluetoothLock.h>
#include <btstack.h>
#include <RCSwitch.h>
#include <stdarg.h>
#include "rf_proto.h"

#define BLE_NAME "eepy-hub"
// Every CYW43 radio burst wrecks the 433 frame the MX-05 is receiving at that
// moment (supply/RF coupling; tested: BLE up but not advertising = fine, the
// stock 30ms advertising = ~0 decodes, even with RX on its own core). So keep
// the radio mostly quiet: advertise once a second, and ask the phone for a
// ~200ms connection interval. A 433 frame is ~56ms and sent 4x, so most codes
// still find a clean window.
#define BLE_ADV_INTERVAL 1600 // x0.625ms = 1s
#define BLE_CONN_MIN 128      // x1.25ms = 160ms
#define BLE_CONN_MAX 192      // x1.25ms = 240ms
#define BLE_CONN_TIMEOUT 400  // x10ms = 4s supervision timeout
// A characteristic notify doesn't wait for the previous one, so back-to-back
// chunks overwrite each other. Send one chunk per connection interval instead,
// sized to the negotiated MTU (20 bytes at the default MTU: a ~120 byte JSON
// line then takes ~1.5s of the 2.5s cadence).
#define BLE_CHUNK_MAX 180
#define BLE_CHUNK_MS 250
// Notifications are dropped until the page subscribes; wait before hello.
#define BLE_HELLO_DELAY_MS 1500

// 433 RX lives on core 1 so BTstack's IRQ work on core 0 can't delay the
// rc-switch edge ISR (~200us tolerance). GPIO IRQs are per-core; valid 32-bit
// codes go to core 0 over the SIO FIFO.
static RCSwitch rx;
static volatile unsigned long ignored = 0; // written by core 1 only

// Nordic UART Service, same UUIDs as the ESP32 probe. Hand-rolled instead of
// arduino-pico's BLEServiceUART: its BLEUUID(String) parse uses sscanf "%llx",
// which newlib-nano lacks, so every 128-bit UUID came out all zeros. It also
// never clears con_handle on disconnect.
static const uint8_t NUS_SVC[16] = {0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
                                    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E};
static const uint8_t NUS_RX[16] = {0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93,
                                   0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E};
static const uint8_t NUS_TX[16] = {0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93,
                                   0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E};

struct HubService : BLEService, BLECharacteristicCallbacks {
  BLECharacteristic rxc{BLEUUID(NUS_RX), BLEWrite | BLEWriteWithoutResponse, "eepy-hub RX"};
  BLECharacteristic txc{BLEUUID(NUS_TX), BLERead | BLENotify, "eepy-hub TX"};
  // Phone -> hub bytes; filled in BT context, drained by loop().
  volatile uint8_t in[64];
  volatile uint8_t in_head = 0, in_tail = 0;

  HubService() : BLEService(BLEUUID(NUS_SVC)) {
    rxc.setCallbacks(this);
    addCharacteristic(&rxc);
    addCharacteristic(&txc);
  }
  bool connected() { return con_handle != 0; }
  uint16_t handle() { return con_handle; }
  int read() {
    if (in_tail == in_head) return -1;
    uint8_t c = in[in_tail];
    in_tail = (in_tail + 1) % sizeof(in);
    return c;
  }
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *d = (const uint8_t *)c->valueData();
    for (size_t i = 0; i < c->valueLen(); i++) {
      uint8_t next = (in_head + 1) % sizeof(in);
      if (next == in_tail) return;
      in[in_head] = d[i];
      in_head = next;
    }
  }

protected:
  void disconnected() override {
    BLEService::disconnected();
    con_handle = 0;
  }
};
static HubService hub;
static unsigned long got = 0;
static unsigned long last_code = 0, last_code_ms = 0;

// Last decoded sample, replayed to a freshly connected phone.
static bool have_last = false;
static uint8_t last_seq, last_idx, last_total, last_ch, last_tag, last_flags;
static int last_rssi;
static unsigned long last_rx_ms = 0;

// SSID reassembled from 433 chunks (rf_proto.h). Samples carry the tag, so a
// complete name stays valid for as long as the tag keeps matching.
static int name_tag = -1;
static uint8_t name_buf[EEPY_SSID_MAX];
static uint16_t name_have = 0; // bit n = chunk n received
static int name_end = -1;      // chunk holding the terminator, -1 = not seen
static bool name_ok = false;
static int name_len = 0;

// BLE TX ring buffer, drained BLE_CHUNK bytes at a time in loop().
static char bq[2048];
static size_t bq_head = 0, bq_tail = 0;

static void bq_push(const char *s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    size_t next = (bq_head + 1) % sizeof(bq);
    if (next == bq_tail) return; // full: drop the rest (phone too slow)
    bq[bq_head] = s[i];
    bq_head = next;
  }
}

// USB-only line (plain chatter or JSON not meant for the phone).
static void usb_line(const char *fmt, ...) {
  char line[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  Serial.println(line);
}

// JSON line to USB serial and, when a phone is connected, BLE.
static void emit(const char *fmt, ...) {
  char line[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if ((size_t)n >= sizeof(line)) n = sizeof(line) - 1;
  Serial.println(line);
  if (hub.connected()) {
    bq_push(line, n);
    bq_push("\n", 1);
  }
}

static void ble_pump() {
  static unsigned long last_chunk = 0;
  if (!hub.connected()) {
    bq_head = bq_tail = 0;
    return;
  }
  if (bq_head == bq_tail || millis() - last_chunk < BLE_CHUNK_MS) return;
  last_chunk = millis();
  int chunk;
  {
    BluetoothLock b;
    chunk = (int)att_server_get_mtu(hub.handle()) - 3;
  }
  if (chunk < 20) chunk = 20;
  if (chunk > BLE_CHUNK_MAX) chunk = BLE_CHUNK_MAX;
  uint8_t buf[BLE_CHUNK_MAX];
  int n = 0;
  while (n < chunk && bq_tail != bq_head) {
    buf[n++] = (uint8_t)bq[bq_tail];
    bq_tail = (bq_tail + 1) % sizeof(bq);
  }
  hub.txc.setValue(buf, n);
}

// The page keys history on bssid; the hub never learns the real one, so the
// SSID tag stands in. ssid is "" until the name has arrived over 433.
static void emit_lock() {
  char ssid[2 * EEPY_SSID_MAX + 1];
  size_t o = 0;
  if (name_ok && name_tag == last_tag) {
    if (!name_len) {
      strcpy(ssid, "<hidden>"); // same as the probe's scan list
      o = strlen(ssid);
    }
    for (int i = 0; i < name_len; i++) {
      uint8_t c = name_buf[i];
      if (c < 0x20) continue;
      if (c == '"' || c == '\\') ssid[o++] = '\\';
      ssid[o++] = (char)c;
    }
  }
  ssid[o] = '\0';
  emit("{\"t\":\"lock\",\"src\":\"rf\",\"i\":%u,\"n\":%u,\"bssid\":\"rf-t%u\","
       "\"ch\":%u,\"tag\":%u,\"ssid\":\"%s\"}",
       last_idx + 1, last_total, last_tag, last_ch, last_tag, ssid);
}

static void name_chunk(uint8_t k, uint8_t b0, uint8_t b1, uint8_t tag) {
  if (tag != name_tag) { // a new name: start over
    name_tag = tag;
    name_have = 0;
    name_end = -1;
    name_ok = false;
  }
  if (name_ok) {
    // Periodic refresh of a name we already have; a mismatch means the tag
    // was reused for a different SSID, so take the new one.
    if ((2 * k >= name_len || name_buf[2 * k] == b0) &&
        (2 * k + 1 >= name_len || name_buf[2 * k + 1] == b1))
      return;
    name_have = 0;
    name_end = -1;
    name_ok = false;
  }
  name_buf[2 * k] = b0;
  name_buf[2 * k + 1] = b1;
  name_have |= 1u << k;
  if ((!b0 || !b1) && name_end < 0) name_end = k;

  uint16_t need = name_end >= 0 ? (uint16_t)((1u << (name_end + 1)) - 1) : 0xFFFF;
  if ((name_have & need) != need) return;
  name_ok = true;
  name_len = (int)strnlen((const char *)name_buf, EEPY_SSID_MAX);
  usb_line("[pico-hub] ssid tag=%d complete (%d bytes)", name_tag, name_len);
  if (have_last && last_tag == name_tag) emit_lock();
}

static void emit_sample() {
  emit("{\"t\":\"s\",\"src\":\"rf\",\"seq\":%u,\"i\":%u,\"n\":%u,"
       "\"rssi\":%d,\"ch\":%u,\"lock\":%d,\"got\":%lu,\"age\":%lu}",
       last_seq, last_idx + 1, last_total, last_rssi, last_ch,
       (last_flags & EEPY_FLAG_LOCK) ? 1 : 0, got,
       (millis() - last_rx_ms) / 1000);
}

static void ble_poll() {
  static bool was_conn = false;
  static unsigned long conn_ms = 0;
  static bool hello_sent = false;
  bool conn = hub.connected();
  if (conn && !was_conn) {
    conn_ms = millis();
    hello_sent = false;
    {
      BluetoothLock b;
      gap_request_connection_parameter_update(hub.handle(), BLE_CONN_MIN, BLE_CONN_MAX,
                                              0, BLE_CONN_TIMEOUT);
    }
    usb_line("[pico-hub] BLE connected");
  } else if (!conn && was_conn) {
    usb_line("[pico-hub] BLE disconnected");
  }
  was_conn = conn;
  if (conn && !hello_sent && millis() - conn_ms >= BLE_HELLO_DELAY_MS) {
    hello_sent = true;
    emit("{\"t\":\"hello\",\"role\":\"hub\",\"got\":%lu}", got);
    if (have_last) {
      emit_lock();
      emit_sample();
    }
  }

  // Commands: the hub can't forward them (433 is ESP32 -> Pico only).
  static char cmd[48];
  static size_t cmd_len = 0;
  int c;
  while ((c = hub.read()) >= 0) {
    if (c == '\n' || c == '\r') {
      if (!cmd_len) continue;
      cmd[cmd_len] = '\0';
      cmd_len = 0;
      if (!strcmp(cmd, "help")) {
        emit("{\"t\":\"help\",\"cmds\":\"hub is read-only (433 is one-way). "
             "connect to eepy-probe to control.\"}");
      } else {
        emit("{\"t\":\"err\",\"msg\":\"%s: hub is read-only, connect to eepy-probe\"}",
             strchr(cmd, '"') ? "cmd" : cmd);
      }
    } else if (cmd_len < sizeof(cmd) - 1) {
      cmd[cmd_len++] = (char)c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }
  Serial.println("[pico-hub] boot, waiting for 433MHz packets...");
  BLE.begin(BLE_NAME);
  BLE.server()->addService(&hub);
  BLE.startAdvertising();
  {
    // startAdvertising() hardcodes a 30ms interval (setter is private);
    // re-apply ours. BTstack keeps it for re-advertising after disconnects.
    BluetoothLock b;
    bd_addr_t none = {0};
    gap_advertisements_enable(0);
    gap_advertisements_set_params(BLE_ADV_INTERVAL, BLE_ADV_INTERVAL, 0, 0, none, 0x07, 0x00);
    gap_advertisements_enable(1);
  }
  Serial.println("[pico-hub] BLE advertising as \"" BLE_NAME "\" (read-only relay)");
  Serial.printf("[pico-hub] rc-switch RX on core 1, GP%d @115200 USB-serial\n", EEPY_PICO_RX_PIN);
  Serial.println("[pico-hub] tip: `pio device monitor -e pico2w` (no JTAG needed)");
}

void setup1() {
  // RP2350 pads start isolated and arduino-pico digitalRead() returns 0 without
  // a pinMode; rc-switch's attachInterrupt does neither. INPUT = no pulls
  // (internal pull-down makes the RP2350-E9 latch worse).
  pinMode(EEPY_PICO_RX_PIN, INPUT);
  rx.setReceiveTolerance(60);
  rx.enableReceive(EEPY_PICO_RX_PIN); // attaches the ISR to this core
}

void loop1() {
  if (!rx.available()) return;
  unsigned long code = rx.getReceivedValue();
  unsigned int bits = rx.getReceivedBitlength();
  if (code == 0 || bits != EEPY_RF_BITLEN) {
    ignored++;
  } else {
    rp2040.fifo.push_nb((uint32_t)code); // drop if core 0 is 8 codes behind
  }
  rx.resetAvailable();
}

void loop() {
  while (rp2040.fifo.available()) {
    uint32_t code = rp2040.fifo.pop();
    if (code == last_code && millis() - last_code_ms < 1000) {
      // ESP32 repeats each code RF_REPEAT times; drop the extra decodes.
      last_code_ms = millis();
    } else {
      last_code = code;
      last_code_ms = millis();
      got++;
      if ((code & 7) == EEPY_FLAG_CHUNK) {
        uint8_t k, b0, b1, tag;
        if (eepy_decode_chunk(code, &k, &b0, &b1, &tag)) name_chunk(k, b0, b1, tag);
        else usb_line("{\"t\":\"rf_bad\",\"why\":\"chunk check\",\"value\":%lu}", code);
        continue;
      }
      uint8_t seq, idx, total, ch, tag, flags;
      int rssi;
      eepy_decode(code, &seq, &idx, &total, &rssi, &ch, &tag, &flags);
      bool new_ap = !have_last || idx != last_idx || total != last_total || ch != last_ch ||
                    tag != last_tag;
      last_seq = seq, last_idx = idx, last_total = total, last_ch = ch;
      last_tag = tag, last_flags = flags, last_rssi = rssi;
      last_rx_ms = millis();
      have_last = true;
      if (new_ap) emit_lock();
      // Same JSON line format as the ESP32's BLE UART samples (src/main.c).
      emit_sample();
    }
  }

  ble_poll();
  ble_pump();

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
    usb_line("{\"t\":\"diag\",\"pin\":%d,\"toggles_10s\":%lu,\"level\":%d,"
             "\"got\":%lu,\"ignored\":%lu}",
             EEPY_PICO_RX_PIN, toggles, lvl, got, ignored);
    toggles = 0;
  }
}
