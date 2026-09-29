#pragma once
#include <stdbool.h>
#include <stddef.h>

// BLE "serial port" on the ESP32 (NimBLE peripheral, Nordic UART Service
// UUIDs so generic apps like nRF Connect / Serial Bluetooth Terminal work).
// Line-oriented: incoming writes are split on '\n' and queued for the main
// loop; outgoing lines are chunked to the negotiated MTU. One client at a time.

#ifdef __cplusplus
extern "C" {
#endif

// Call after nvs_flash_init (wifi_init_sta does it).
void ble_uart_init(const char *name);

// A client is connected and subscribed to notifications.
bool ble_uart_ready(void);

// Pop one received command line (without '\n'). Non-blocking.
bool ble_uart_read(char *buf, size_t n);

// Send one line; '\n' is appended. Dropped silently if not ready.
void ble_uart_send(const char *line);

#ifdef __cplusplus
}
#endif
