#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

static const char *TAG = "wifiscan";

// Sanity: 256 glyphs x 5 bytes (catches font table corruption)
_Static_assert(sizeof(font5x7) == 1280, "font5x7 must be 256x5 bytes");

// ---- 0.42" OLED: SSD1306-compatible, 72x40, I2C 0x3C, column offset 28 ----
#define OLED_ADDR   0x3C
#define OLED_WIDTH  72
#define OLED_HEIGHT 40
#define OLED_COL_OFFSET 28
#define OLED_PAGES  (OLED_HEIGHT / 8)  // 5

#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 400000

// ---- 5x7 text: 6px wide (5+spacing) x 8px tall -> 12 cols x 5 rows ----
#define CHAR_W 6
#define CHAR_H 8
#define TEXT_COLS (OLED_WIDTH / CHAR_W)    // 12
#define TEXT_ROWS (OLED_HEIGHT / CHAR_H)   // 5

#define MAX_APS 20
#define DWELL_MS 2500

static uint8_t fb[OLED_WIDTH * OLED_PAGES];

// ---------- OLED low level ----------

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
        .scan_time = { .active = { .min = 100, .max = 300 } },
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

// One screen per network: header "i/n -52dBm[*]" + SSID wrapped over rows 1..4.
static void show_ap(int idx, int total, const wifi_ap_record_t *ap) {
    char hdr[32], ssid[37];
    const char *lines[TEXT_ROWS];
    ssid_to_str(ap, ssid, sizeof(ssid));

    // '*' marks encrypted networks ("1/20 -99*" fits 12 cols)
    snprintf(hdr, sizeof(hdr), "%d/%d %d%s",
             idx + 1, total, ap->rssi,
             ap->authmode == WIFI_AUTH_OPEN ? "" : "*");
    lines[0] = hdr;

    // Wrap SSID into 12-char rows (static buffers survive the call)
    static char rows[4][13];
    size_t len = strlen(ssid);
    int r;
    for (r = 0; r < 4; r++) {
        size_t off = (size_t)r * 12;
        if (off >= len) break;
        size_t n = len - off > 12 ? 12 : len - off;
        memcpy(rows[r], ssid + off, n);
        rows[r][n] = '\0';
        lines[1 + r] = rows[r];
    }
    for (; r < 4; r++) lines[1 + r] = NULL;

    show_lines(lines);
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-C3 wifi scanner on 0.42\" OLED (72x40)");

    bool ok = i2c_try_pins(5, 6);
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
    ESP_LOGI(TAG, "wifi started, scanning...");

    static wifi_ap_record_t aps[MAX_APS];

    while (1) {
        show_msg("Scanning", "...");
        int n = wifi_scan(aps, MAX_APS);

        if (n < 0) {
            ESP_LOGW(TAG, "scan failed, retrying");
            show_msg("Scan", "failed...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        if (n == 0) {
            ESP_LOGI(TAG, "no networks found");
            show_msg("No networks", "retry...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        ESP_LOGI(TAG, "found %d networks:", n);
        for (int i = 0; i < n; i++) {
            char ssid[37];
            ssid_to_str(&aps[i], ssid, sizeof(ssid));
            ESP_LOGI(TAG, "  %d/%d rssi=%d ch=%d %s", i + 1, n,
                     aps[i].rssi, aps[i].primary, ssid);
        }

        // Walk the cache one screen at a time, then loop back to rescan.
        for (int i = 0; i < n; i++) {
            show_ap(i, n, &aps[i]);
            vTaskDelay(pdMS_TO_TICKS(DWELL_MS));
        }
        // End of list -> rescan (picks up new/gone networks) and wrap to 1.
    }
}
