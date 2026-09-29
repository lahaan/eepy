// NimBLE Nordic UART Service peripheral (see ble_uart.h).

#include <string.h>
#include "ble_uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "bleuart";

#define RX_LINE_MAX    96
#define RX_QUEUE    8
#define ATT_HDR     3
#define NOTIFY_TRIES   25
#define NOTIFY_WAIT_MS 20
// 100-200ms connection interval: few radio slots, leaves airtime for sniffing
#define CONN_ITVL_MIN 80  // x1.25ms
#define CONN_ITVL_MAX 160
#define CONN_TIMEOUT  400 // x10ms
// ~0.5s advertising (default is ~30ms): phones still find us in about a
// second, and the shared radio stays on the sniffed channel far more
#define ADV_ITVL_MIN  800 // x0.625ms
#define ADV_ITVL_MAX  880

// NUS UUIDs 6E40000x-B5A3-F393-E0A9-E50E24DCCA9E (little-endian bytes)
#define NUS_UUID(x) BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
                                     0x93, 0xf3, 0xa3, 0xb5, (x), 0x00, 0x40, 0x6e)
static const ble_uuid128_t svc_uuid = NUS_UUID(0x01);
static const ble_uuid128_t rx_uuid = NUS_UUID(0x02); // client -> us (write)
static const ble_uuid128_t tx_uuid = NUS_UUID(0x03); // us -> client (notify)

static uint16_t tx_handle;
static uint16_t conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool subscribed;
static volatile uint16_t mtu = 23;
static uint8_t own_addr_type;
static QueueHandle_t rxq;
static char rx_line[RX_LINE_MAX];
static size_t rx_len;

static void advertise(void);

// Split writes into lines; a write without '\n' is also taken as one line
// (terminal apps differ on whether they send line endings).
static int rx_access(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
  uint8_t buf[256];
  uint16_t len = 0;
  if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  for (uint16_t i = 0; i <= len; i++) {
    bool end = i == len;
    char c = end ? '\n' : (char)buf[i];
    if (c == '\r' || c == '\n') {
      if (rx_len) {
        rx_line[rx_len] = '\0';
        xQueueSend(rxq, rx_line, 0);
        rx_len = 0;
      }
    } else if (rx_len < RX_LINE_MAX - 1) {
      rx_line[rx_len++] = c;
    }
  }
  return 0;
}

static int tx_access(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
  return 0; // notify-only, nothing to read
}

static const struct ble_gatt_svc_def svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &rx_uuid.u,
                .access_cb = rx_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &tx_uuid.u,
                .access_cb = tx_access,
                .val_handle = &tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static int gap_event(struct ble_gap_event *ev, void *arg) {
  switch (ev->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (ev->connect.status != 0) {
      advertise();
      break;
    }
    conn = ev->connect.conn_handle;
    mtu = 23;
    ESP_LOGI(TAG, "connected");
    ble_gattc_exchange_mtu(conn, NULL, NULL);
    struct ble_gap_upd_params p = {
        .itvl_min = CONN_ITVL_MIN,
        .itvl_max = CONN_ITVL_MAX,
        .latency = 0,
        .supervision_timeout = CONN_TIMEOUT,
    };
    ble_gap_update_params(conn, &p);
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "disconnected (reason 0x%x)", ev->disconnect.reason);
    conn = BLE_HS_CONN_HANDLE_NONE;
    subscribed = false;
    advertise();
    break;
  case BLE_GAP_EVENT_SUBSCRIBE:
    if (ev->subscribe.attr_handle == tx_handle) {
      subscribed = ev->subscribe.cur_notify;
      ESP_LOGI(TAG, "notify %s", subscribed ? "on" : "off");
    }
    break;
  case BLE_GAP_EVENT_MTU:
    mtu = ev->mtu.value;
    ESP_LOGI(TAG, "mtu %u", mtu);
    break;
  case BLE_GAP_EVENT_ADV_COMPLETE:
    advertise();
    break;
  default:
    break;
  }
  return 0;
}

// Adv packet: flags + NUS UUID (fits 31 bytes); name goes in scan response.
static void advertise(void) {
  struct ble_hs_adv_fields f = {0};
  f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  f.uuids128 = (ble_uuid128_t *)&svc_uuid;
  f.num_uuids128 = 1;
  f.uuids128_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&f);

  struct ble_hs_adv_fields rsp = {0};
  const char *name = ble_svc_gap_device_name();
  rsp.name = (uint8_t *)name;
  rsp.name_len = strlen(name);
  rsp.name_is_complete = 1;
  if (rc == 0) rc = ble_gap_adv_rsp_set_fields(&rsp);

  struct ble_gap_adv_params ap = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
      .itvl_min = ADV_ITVL_MIN,
      .itvl_max = ADV_ITVL_MAX,
  };
  if (rc == 0) {
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &ap, gap_event, NULL);
  }
  if (rc != 0) ESP_LOGE(TAG, "advertise failed rc=%d", rc);
}

static void on_sync(void) {
  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &own_addr_type);
  advertise();
  ESP_LOGI(TAG, "advertising as \"%s\"", ble_svc_gap_device_name());
}

static void on_reset(int reason) {
  ESP_LOGW(TAG, "host reset, reason %d", reason);
}

static void host_task(void *arg) {
  nimble_port_run(); // returns only on nimble_port_stop()
  nimble_port_freertos_deinit();
}

void ble_uart_init(const char *name) {
  rxq = xQueueCreate(RX_QUEUE, RX_LINE_MAX);
  esp_err_t r = nimble_port_init();
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(r));
    return;
  }
  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.reset_cb = on_reset;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_gatts_count_cfg(svcs);
  ble_gatts_add_svcs(svcs);
  ble_svc_gap_device_name_set(name);

  nimble_port_freertos_init(host_task);
}

bool ble_uart_ready(void) {
  return conn != BLE_HS_CONN_HANDLE_NONE && subscribed;
}

bool ble_uart_read(char *buf, size_t n) {
  char line[RX_LINE_MAX];
  if (!rxq || xQueueReceive(rxq, line, 0) != pdTRUE) return false;
  strncpy(buf, line, n - 1);
  buf[n - 1] = '\0';
  return true;
}

// One notification. Bursts (the AP list) can run the host out of buffers at
// a slow connection interval, so wait for them to drain instead of dropping.
static bool notify(const char *data, size_t len) {
  for (int tries = 0; tries < NOTIFY_TRIES; tries++) {
    if (!ble_uart_ready()) return false;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) {
      int rc = ble_gatts_notify_custom(conn, tx_handle, om); // consumes om
      if (rc == 0) return true;
      if (rc != BLE_HS_ENOMEM) return false;
    }
    vTaskDelay(pdMS_TO_TICKS(NOTIFY_WAIT_MS));
  }
  return false;
}

void ble_uart_send(const char *line) {
  char buf[256];
  size_t len = strlen(line);
  if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
  memcpy(buf, line, len);
  buf[len++] = '\n';
  size_t chunk = mtu - ATT_HDR;
  for (size_t off = 0; off < len; off += chunk) {
    size_t n = len - off < chunk ? len - off : chunk;
    if (!notify(buf + off, n)) return;
  }
}
