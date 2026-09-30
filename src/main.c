#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "font5x7.h"
#include "rf_proto.h"
#include "rf_tx.h"
#include "sniff.h"
#include "ble_uart.h"
#include "esp_mac.h"

static const char *TAG = "wifiscan";

// Sanity: 256 glyphs x 5 bytes (catches font table corruption)
_Static_assert(sizeof(font5x7) == 1280, "font5x7 must be 256x5 bytes");

// 0.42" OLED: SSD1306?, 72x40, I2C 0x3C, column offset 28
#define OLED_ADDR   0x3C
#define OLED_WIDTH  72
#define OLED_HEIGHT 40
#define OLED_COL_OFFSET 28
#define OLED_PAGES  (OLED_HEIGHT / 8)  // 5

#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 400000

// 5x7 text: 6px wide (5+spacing) x 8px tall -> 12 cols x 5 rows
#define CHAR_W 6
#define CHAR_H 8
#define TEXT_COLS (OLED_WIDTH / CHAR_W)    // 12
#define TEXT_ROWS (OLED_HEIGHT / CHAR_H)   // 5

#define MAX_APS 20
#define LOOP_MS      50
#define WINDOW_MS    500   // RSSI averaging window / OLED refresh
#define STALE_MS     2000  // no beacons this long -> show "---"
#define LOST_MS      10000 // no beacons this long -> rescan + relock
#define RF_PERIOD_MS 2500  // one 433MHz code (~224ms on air) -> ~9% duty (EU SRD <=10%)
#define NAME_REPEAT_MS  30000  // SSID over 433: second pass after the first
#define NAME_REFRESH_MS 300000 // then only for a Pico that missed it / rebooted

#define BTN_GPIO    9      // BOOT button: short = next AP, long = rescan
#define BTN_LONG_MS 800

#define BLE_NAME "eepy-probe"

static uint8_t fb[OLED_WIDTH * OLED_PAGES];

// OLED

static esp_err_t oled_write_cmd(uint8_t cmd) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x00, true); // Co=0, D/C=0 -> command
    i2c_master_write_byte(h, cmd, true);
    i2c_master_stop(h);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
    return r;
}

static esp_err_t oled_init(void) {
    // u8g2 72x40 ER sequence (EastRising 0.42")
    const uint8_t seq[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x27,       // 40-line mux
        0xD3, 0x00,
        0xAD, 0x30,       // internal IREF (brightness on 0.42")
        0x8D, 0x14,       // charge pump on
        0x40,             // start line 0
        0xA6,             // normal display
        0xA4,             // RAM content
        0x20, 0x00,       // horizontal addressing
        0xA1,             // segment remap
        0xC8,             // COM scan reverse
        0xDA, 0x12,
        0x81, 0xAF,       // contrast
        0xD9, 0x22,
        0xDB, 0x20,
        0x2E,             // scroll off
        0xAF,             // on
    };
    for (size_t i = 0; i < sizeof(seq); i++) {
        esp_err_t r = oled_write_cmd(seq[i]);
        if (r != ESP_OK) return r;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static void oled_clear(void) {
    memset(fb, 0x00, sizeof(fb));
}

static inline void set_pixel(int x, int y) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    fb[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
}

static esp_err_t oled_flush(void) {
    esp_err_t r;
    r = oled_write_cmd(0x21); if (r) return r;
    r = oled_write_cmd(OLED_COL_OFFSET); if (r) return r;
    r = oled_write_cmd(OLED_COL_OFFSET + OLED_WIDTH - 1); if (r) return r;
    r = oled_write_cmd(0x22); if (r) return r;
    r = oled_write_cmd(0x00); if (r) return r;
    r = oled_write_cmd(OLED_PAGES - 1); if (r) return r;

    const size_t CHUNK = 32;
    size_t offset = 0;
    while (offset < sizeof(fb)) {
        size_t n = sizeof(fb) - offset;
        if (n > CHUNK) n = CHUNK;
        i2c_cmd_handle_t h = i2c_cmd_link_create();
        i2c_master_start(h);
        i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(h, 0x40, true); // data
        for (size_t i = 0; i < n; i++) {
            i2c_master_write_byte(h, fb[offset + i], true);
        }
        i2c_master_stop(h);
        r = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(h);
        if (r != ESP_OK) return r;
        offset += n;
    }
    return ESP_OK;
}

static bool i2c_probe_addr(uint8_t addr) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(h);
    esp_err_t r = i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(h);
    return r == ESP_OK;
}

static bool i2c_try_pins(int sda, int scl) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    if (i2c_param_config(I2C_PORT, &cfg) != ESP_OK) return false;
    if (i2c_driver_install(I2C_PORT, cfg.mode, 0, 0, 0) != ESP_OK) return false;

    bool found = i2c_probe_addr(OLED_ADDR) || i2c_probe_addr(0x3D);
    ESP_LOGI(TAG, "probe SDA=%d SCL=%d -> %s", sda, scl, found ? "OLED ACK" : "no ack");
    if (!found) i2c_driver_delete(I2C_PORT);
    return found;
}

// ---------- text ----------

static void draw_char(int x, int y, char c) {
    const uint8_t *g = &font5x7[(uint8_t)c * 5];
    for (int col = 0; col < 5; col++) {
        if (x + col >= OLED_WIDTH) break;
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) set_pixel(x + col, y + row);
        }
    }
}

// Draw string starting at pixel (x,y); stops at right edge. No wrapping.
static void draw_string(int x, int y, const char *s) {
    while (*s && x + 5 <= OLED_WIDTH) {
        draw_char(x, y, *s++);
        x += CHAR_W;
    }
}

// Draw one full text screen: up to TEXT_ROWS lines, each truncated to TEXT_COLS.
static void show_lines(const char *lines[TEXT_ROWS]) {
    oled_clear();
    for (int r = 0; r < TEXT_ROWS; r++) {
        if (lines[r]) draw_string(0, r * CHAR_H, lines[r]);
    }
    oled_flush();
}

static void show_msg(const char *l0, const char *l1) {
    const char *lines[TEXT_ROWS] = { l0, l1, NULL, NULL, NULL };
    show_lines(lines);
}

// ---------- wifi scan ----------

static void wifi_init_sta(void) {
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(r);
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Blocking scan, results sorted strongest-first. Returns count or -1 on error.
static int wifi_scan(wifi_ap_record_t *out, int max) {
    wifi_scan_config_t sc = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,          // all channels
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        // scan_time left 0 = driver default; custom times are rejected (and the
        // scan drags to ~9s) when BT is enabled, as the driver warns.
    };
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) return -1;
    uint16_t n = 0;
    if (esp_wifi_scan_get_ap_num(&n) != ESP_OK) return -1;
    if (n == 0) return 0;
    if (n > (uint16_t)max) n = (uint16_t)max;
    uint16_t got = n;
    if (esp_wifi_scan_get_ap_records(&got, out) != ESP_OK) return -1;
    for (int i = 0; i < got; i++) {
        for (int j = i + 1; j < got; j++) {
            if (out[j].rssi > out[i].rssi) {
                wifi_ap_record_t t = out[i];
                out[i] = out[j];
                out[j] = t;
            }
        }
    }
    return got;
}

// Copy SSID bytes to nul-terminated string; "<hidden>" if empty.
static void ssid_to_str(const wifi_ap_record_t *ap, char *dst, size_t dstsz) {
    size_t len = strnlen((const char *)ap->ssid, sizeof(ap->ssid));
    if (len == 0) {
        snprintf(dst, dstsz, "<hidden>");
        return;
    }
    if (len > dstsz - 1) len = dstsz - 1;
    memcpy(dst, ap->ssid, len);
    dst[len] = '\0';
}


// ---------- lock mode ----------

static inline uint32_t now_ms(void) {
    return (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
}

// 5x7 glyph doubled to 10x14.
static void draw_char_2x(int x, int y, char c) {
    const uint8_t *g = &font5x7[(uint8_t)c * 5];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (!(bits & (1 << row))) continue;
            int px = x + col * 2, py = y + row * 2;
            set_pixel(px, py);
            set_pixel(px + 1, py);
            set_pixel(px, py + 1);
            set_pixel(px + 1, py + 1);
        }
    }
}

// Lock screen:
//   SSID
//   c6 1/5
//   -52dBm      (2x font)
//   [bar]  10/s (beacons per second)
static void show_lock(const wifi_ap_record_t *ap, int idx, int total,
                      bool have, int rssi, int rate) {
    char ssid[37], line[40];
    oled_clear();
    ssid_to_str(ap, ssid, sizeof(ssid));
    draw_string(0, 0, ssid);
    snprintf(line, sizeof(line), "c%d %d/%d", ap->primary, idx + 1, total);
    draw_string(0, 8, line);

    int x = 0;
    if (have) {
        snprintf(line, sizeof(line), "%d", rssi);
        for (const char *s = line; *s; s++, x += 12) draw_char_2x(x, 17, *s);
        draw_string(x, 24, "dBm");
        // -95dBm = empty, -35dBm = full 44px
        int w = (rssi + 95) * 44 / 60;
        if (w < 0) w = 0;
        if (w > 44) w = 44;
        for (int bx = 0; bx < w; bx++) {
            for (int by = 33; by < 39; by++) set_pixel(bx, by);
        }
    } else {
        for (const char *s = "---"; *s; s++, x += 12) draw_char_2x(x, 17, *s);
    }
    snprintf(line, sizeof(line), "%d/s", rate);
    draw_string(48, 32, line);
    oled_flush();
}

static void button_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

// Polled from the main loop. Returns 1 on short press, 2 on long, else 0.
static int button_poll(void) {
    static bool down = false;
    static uint32_t down_at;
    bool pressed = gpio_get_level((gpio_num_t)BTN_GPIO) == 0;
    if (pressed && !down) {
        down = true;
        down_at = now_ms();
    } else if (!pressed && down) {
        down = false;
        return now_ms() - down_at >= BTN_LONG_MS ? 2 : 1;
    }
    return 0;
}

static wifi_ap_record_t aps[MAX_APS];
static int n_aps, cur;

// Live sample state, reset whenever the target changes.
static bool have;
static int rssi, rate;
static uint32_t last_win, last_rx, last_rf;
static uint8_t rf_seq;

// ---------- JSON lines (BLE out; the Pico prints the same format) ----------
//   {"t":"ap","i":1,"n":3,"bssid":"..","ch":1,"rssi":-52,"enc":1,"ssid":".."}
//   {"t":"lock","i":1,"n":3,"bssid":"..","ch":1,"ssid":".."}
//   {"t":"s","src":"ble","seq":12,"i":1,"n":3,"rssi":-52,"rate":9,"ch":1}
//   {"t":"ok","cmd":".."} / {"t":"err","msg":".."}
// Indices are 1-based, matching the OLED and the `lock N` command.

// SSID as a JSON string body: escape quotes/backslashes, drop control chars.
static void json_ssid(const wifi_ap_record_t *ap, char *dst, size_t n) {
    char ssid[37];
    ssid_to_str(ap, ssid, sizeof(ssid));
    size_t o = 0;
    for (const char *s = ssid; *s && o + 2 < n; s++) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x20) continue;
        if (c == '"' || c == '\\') dst[o++] = '\\';
        dst[o++] = (char)c;
    }
    dst[o] = '\0';
}

static void send_ap(int i) {
    char ssid[80], line[200];
    json_ssid(&aps[i], ssid, sizeof(ssid));
    snprintf(line, sizeof(line),
             "{\"t\":\"ap\",\"i\":%d,\"n\":%d,\"bssid\":\"" MACSTR "\",\"ch\":%d,"
             "\"rssi\":%d,\"enc\":%d,\"ssid\":\"%s\"}",
             i + 1, n_aps, MAC2STR(aps[i].bssid), aps[i].primary, aps[i].rssi,
             aps[i].authmode != WIFI_AUTH_OPEN, ssid);
    ble_uart_send(line);
}

static void send_lock(void) {
    char ssid[80], line[200];
    json_ssid(&aps[cur], ssid, sizeof(ssid));
    snprintf(line, sizeof(line),
             "{\"t\":\"lock\",\"i\":%d,\"n\":%d,\"bssid\":\"" MACSTR "\",\"ch\":%d,"
             "\"ssid\":\"%s\"}",
             cur + 1, n_aps, MAC2STR(aps[cur].bssid), aps[cur].primary, ssid);
    ble_uart_send(line);
}

static void send_list(void) {
    for (int i = 0; i < n_aps; i++) send_ap(i);
    if (n_aps) send_lock();
}

static void send_sample(void) {
    char line[160], r[8];
    if (have) snprintf(r, sizeof(r), "%d", rssi);
    else strcpy(r, "null");
    snprintf(line, sizeof(line),
             "{\"t\":\"s\",\"src\":\"ble\",\"seq\":%u,\"i\":%d,\"n\":%d,"
             "\"rssi\":%s,\"rate\":%d,\"ch\":%d}",
             rf_seq, cur + 1, n_aps, r, rate, aps[cur].primary);
    ble_uart_send(line);
}

static void send_status(const char *type, const char *key, const char *val) {
    char line[160];
    snprintf(line, sizeof(line), "{\"t\":\"%s\",\"%s\":\"%s\"}", type, key, val);
    ble_uart_send(line);
}

// ---------- SSID over 433 (see rf_proto.h) ----------
// The name goes out in 2-byte chunks on lock, once more after NAME_REPEAT_MS,
// then every NAME_REFRESH_MS; samples carry name_tag in between so the Pico
// can trust its cached copy. Chunks take every other RF slot while pending.

static uint8_t name_tag;
static uint8_t name_bytes[EEPY_SSID_MAX];
static int name_len;
static uint8_t name_bssid[6];
static bool name_set;
static int name_chunk = -1; // next chunk of the current pass, -1 = idle
static int name_passes;
static uint32_t name_next_ms;
static bool rf_chunk_turn;
static uint8_t recent_tags[4];
static int recent_n;

static int name_chunks(void) {
    return name_len >= (int)EEPY_SSID_MAX ? (int)EEPY_SSID_CHUNKS : name_len / 2 + 1;
}

static void name_start(const wifi_ap_record_t *ap) {
    if (name_set && memcmp(name_bssid, ap->bssid, 6) == 0) return; // relock, Pico has it
    // FNV-1a over BSSID + SSID, folded to the tag width.
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) h = (h ^ ap->bssid[i]) * 16777619u;
    name_len = (int)strnlen((const char *)ap->ssid, EEPY_SSID_MAX);
    for (int i = 0; i < name_len; i++) h = (h ^ ap->ssid[i]) * 16777619u;
    uint8_t tag = (uint8_t)((h ^ (h >> 5) ^ (h >> 10) ^ (h >> 15) ^ (h >> 20)) & EEPY_TAG_MASK);
    // Never reuse a recent tag, so a Pico holding an older name can't match.
    for (bool clash = true; clash;) {
        clash = false;
        for (int i = 0; i < recent_n; i++) {
            if (recent_tags[i] == tag) {
                tag = (tag + 1) & EEPY_TAG_MASK;
                clash = true;
            }
        }
    }
    memmove(recent_tags + 1, recent_tags, sizeof(recent_tags) - 1);
    recent_tags[0] = tag;
    if (recent_n < (int)sizeof(recent_tags)) recent_n++;

    name_tag = tag;
    memcpy(name_bytes, ap->ssid, name_len);
    memcpy(name_bssid, ap->bssid, 6);
    name_set = true;
    name_chunk = 0;
    name_passes = 0;
}

static bool name_pending(uint32_t t) {
    if (name_chunk < 0 && name_set && (int32_t)(t - name_next_ms) >= 0) name_chunk = 0;
    return name_chunk >= 0;
}

static uint32_t name_next_code(uint32_t t, int *chunk) {
    int k = name_chunk;
    uint8_t b0 = 2 * k < name_len ? name_bytes[2 * k] : 0;
    uint8_t b1 = 2 * k + 1 < name_len ? name_bytes[2 * k + 1] : 0;
    *chunk = k;
    if (++name_chunk >= name_chunks()) {
        name_chunk = -1;
        name_passes++;
        name_next_ms = t + (name_passes == 1 ? NAME_REPEAT_MS : NAME_REFRESH_MS);
    }
    return eepy_encode_chunk((uint8_t)k, b0, b1, name_tag);
}

// ---------- targeting ----------

static void reset_sample(void) {
    have = false;
    rate = 0;
    sniff_take(NULL);
    last_win = last_rx = now_ms();
}

static void lock_ap(int i) {
    cur = i;
    esp_err_t r = sniff_start(aps[i].bssid, aps[i].primary);
    char ssid[37];
    ssid_to_str(&aps[i], ssid, sizeof(ssid));
    ESP_LOGI(TAG, "lock %d/%d %s " MACSTR " ch%d (%s)", i + 1, n_aps, ssid,
             MAC2STR(aps[i].bssid), aps[i].primary, esp_err_to_name(r));
    reset_sample();
    name_start(&aps[i]);
    show_lock(&aps[cur], cur, n_aps, have, rssi, rate);
    send_lock();
}

// Full scan, then lock onto `want` if it's still around, else the strongest.
static void scan_and_lock(const uint8_t *want) {
    sniff_stop();
    show_msg("Scanning", "...");
    int n = wifi_scan(aps, MAX_APS);
    if (n <= 0) {
        n_aps = 0;
        ESP_LOGW(TAG, "scan: %s", n < 0 ? "failed" : "no networks");
        show_msg(n < 0 ? "Scan failed" : "No networks", "retry...");
        send_status("err", "msg", n < 0 ? "scan failed" : "no networks");
        return;
    }
    n_aps = n;
    ESP_LOGI(TAG, "found %d networks:", n);
    int pick = 0; // sorted strongest-first
    for (int i = 0; i < n; i++) {
        char ssid[37];
        ssid_to_str(&aps[i], ssid, sizeof(ssid));
        ESP_LOGI(TAG, "  %d/%d rssi=%d ch=%d " MACSTR " %s", i + 1, n,
                 aps[i].rssi, aps[i].primary, MAC2STR(aps[i].bssid), ssid);
        if (want && memcmp(aps[i].bssid, want, 6) == 0) pick = i;
        send_ap(i);
    }
    lock_ap(pick);
}

// `lock` argument: 1-based index or BSSID (aa:bb:cc:dd:ee:ff).
static int find_target(const char *arg) {
    unsigned b[6];
    if (sscanf(arg, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
        for (int i = 0; i < n_aps; i++) {
            bool eq = true;
            for (int k = 0; k < 6; k++) eq &= aps[i].bssid[k] == b[k];
            if (eq) return i;
        }
        return -1;
    }
    char *end;
    long idx = strtol(arg, &end, 10);
    if (end == arg || *end || idx < 1 || idx > n_aps) return -1;
    return (int)idx - 1;
}

// Commands (BLE UART): scan | auto | list | next | lock <n|bssid> | help
static void handle_cmd(char *line) {
    char *arg = line;
    while (*arg && *arg != ' ') arg++;
    if (*arg) *arg++ = '\0';
    while (*arg == ' ') arg++;
    for (char *c = line; *c; c++) *c = (char)tolower((unsigned char)*c);
    ESP_LOGI(TAG, "cmd: %s %s", line, arg);

    if (!strcmp(line, "help")) {
        send_status("help", "cmds", "scan|auto|list|next|lock <n|bssid>");
    } else if (!strcmp(line, "list")) {
        send_list();
    } else if (!strcmp(line, "scan")) {
        uint8_t want[6];
        memcpy(want, aps[cur].bssid, 6);
        scan_and_lock(n_aps ? want : NULL);
    } else if (!strcmp(line, "auto")) {
        scan_and_lock(NULL);
    } else if (!strcmp(line, "next")) {
        if (n_aps) lock_ap((cur + 1) % n_aps);
    } else if (!strcmp(line, "lock")) {
        int i = find_target(arg);
        if (i < 0) {
            send_status("err", "msg", "lock: no such AP (use list)");
            return;
        }
        lock_ap(i);
    } else {
        send_status("err", "msg", "unknown command (try help)");
        return;
    }
    send_status("ok", "cmd", line);
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-C3 AP lock / RSSI probe on 0.42\" OLED (72x40)");

    bool ok = i2c_try_pins(5, 6);
    bool btn_ok = ok; // OLED on (8,9) would steal the BOOT button pin
    if (!ok) ok = i2c_try_pins(8, 9);
    if (!ok) {
        ESP_LOGE(TAG, "No OLED at 0x3C on (5,6) or (8,9)");
        return;
    }
    if (oled_init() != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed");
        return;
    }

    show_msg("WiFi scan", "starting...");
    wifi_init_sta();
    ble_uart_init(BLE_NAME);
    rf_tx_init();
    if (btn_ok) button_init();
    ESP_LOGI(TAG, "TX on GPIO%d, BOOT button GPIO%d %s", EEPY_ESP32_TX_PIN,
             BTN_GPIO, btn_ok ? "(short=next AP, long=rescan)" : "disabled (I2C)");

    last_rf = now_ms();
    bool ble_was_ready = false;
    char cmd[96];

    scan_and_lock(NULL);
    while (1) {
        if (n_aps == 0) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            scan_and_lock(NULL);
            continue;
        }

        // New BLE client: give it the AP list and current target.
        bool ble_ready = ble_uart_ready();
        if (ble_ready && !ble_was_ready) send_list();
        ble_was_ready = ble_ready;

        while (ble_uart_read(cmd, sizeof(cmd))) handle_cmd(cmd);
        if (n_aps == 0) continue; // a command's rescan found nothing

        int b = btn_ok ? button_poll() : 0;
        if (b == 1) {
            lock_ap((cur + 1) % n_aps);
        } else if (b == 2) {
            scan_and_lock(NULL);
        }
        if (b) continue;

        uint32_t t = now_ms();
        if (t - last_win >= WINDOW_MS) {
            int mean;
            int cnt = sniff_take(&mean);
            rate = (int)(cnt * 1000u / (t - last_win));
            last_win = t;
            if (cnt) {
                rssi = mean;
                have = true;
                last_rx = t;
            } else if (t - last_rx > STALE_MS) {
                have = false;
            }
            show_lock(&aps[cur], cur, n_aps, have, rssi, rate);
            send_sample();
        }

        if (t - last_rx >= LOST_MS) {
            // Out of range or changed channel: rescan, stay on it if seen.
            uint8_t want[6];
            memcpy(want, aps[cur].bssid, 6);
            ESP_LOGW(TAG, "no beacons for %ds, rescanning", LOST_MS / 1000);
            scan_and_lock(want);
            continue;
        }

        if (t - last_rf >= RF_PERIOD_MS) {
            if (name_pending(t) && (!have || rf_chunk_turn)) {
                last_rf = t;
                int k;
                rf_tx_send(name_next_code(t, &k));
                rf_chunk_turn = false;
                ESP_LOGI(TAG, "TX name tag=%u chunk %d/%d (len %d)", name_tag, k + 1,
                         name_chunks(), name_len);
            } else if (have) {
                last_rf = t;
                uint32_t code = eepy_encode(rf_seq, (uint8_t)cur, (uint8_t)n_aps, rssi,
                                            aps[cur].primary, name_tag, EEPY_FLAG_LOCK);
                rf_tx_send(code);
                rf_chunk_turn = true;
                ESP_LOGI(TAG, "TX seq=%u lock %d/%d rssi=%d %d/s ch=%d tag=%u", rf_seq,
                         cur + 1, n_aps, rssi, rate, aps[cur].primary, name_tag);
                rf_seq++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
