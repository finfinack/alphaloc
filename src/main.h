#include <stdio.h>
#include <string.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_timer.h"

static const char *TAG = "MAIN";
static const char *TAG_BLE_INIT = "BLE_INIT";
static const char *TAG_BLE_SCAN = "BLE_SCAN";
static const char *TAG_BLE_BOND = "BLE_BOND";
static const char *TAG_BLE_CONN = "BLE_CONN";

esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND; // bonding with peer device after authentication
esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;              // set the IO capability to No output No input
uint8_t key_size = 16;                                 // the key size should be 7~16 bytes
uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
uint8_t oob_support = ESP_BLE_OOB_DISABLE;

#define BLE_NAME "AlphaLoc"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0

#define BLE_SCAN_TIME 5                           // the unit of the duration is second, 0 means scan permanently
#define BLE_RESCAN_PERIOD_WHEN_NOT_CONNECTED 3000 // ms
#define BLE_LOCATION_RETRANSMIT 5000              // ms

#define SONY_COMPANY_ID 0x012D        // Sony Corporation Company Identifier
#define SONY_CAMERA_PRODUCT 0x0003    // Sony Camera
#define BLE_CAM_MAC_PREFIX_DYNAMIC "" // Optional MAC address prefix, e.g. "A4:C1:38"
#define BLE_CAM_NAME "SonyA7V"        // Optional camera name, e.g. "SonyA7"

uint8_t unlock_value = 0x01;

static bool connect = false;
static bool connected = false;
static bool get_location_service = false;
static bool get_remote_service = false;

static esp_gattc_char_elem_t *char_elem_result = NULL;

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

struct gattc_profile_inst
{
  esp_gattc_cb_t gattc_cb;
  uint16_t gattc_if;
  uint16_t app_id;
  uint16_t conn_id;

  uint16_t location_service_start_handle;
  uint16_t location_service_end_handle;
  uint16_t feature_char_handle;
  uint16_t location_char_handle;
  uint16_t notify_char_handle;

  uint16_t remote_service_start_handle;
  uint16_t remote_service_end_handle;
  uint16_t lock_char_handle;
  uint16_t update_char_handle;

  esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

//
// Location Service
//
static esp_bt_uuid_t CAM_LOCATION_SERVICE = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xDD, 0x00, 0xDD, 0x00, 0x80}}};

// WRITE
#define CAM_LOCATION_SERVICE_INFO_CHAR_UUID 0xDD11
static esp_gatt_srvc_id_t CAM_LOCATION_SERVICE_INFO_CHAR = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {
                .uuid16 = CAM_LOCATION_SERVICE_INFO_CHAR_UUID,
            },
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

// READ, 7 byte buffer containing Tz and DST offset config (byte 4)
// Reading this also triggers the secure connection and bonding if needed.
#define CAM_LOCATION_SERVICE_FEATURE_CHAR_UUID 0xDD21
static esp_gatt_srvc_id_t CAM_LOCATION_SERVICE_FEATURE_CHAR = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {
                .uuid16 = CAM_LOCATION_SERVICE_FEATURE_CHAR_UUID,
            },
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

// READ, WRITE, Lock location characteristic:           0x00 - Off, 0x01 - On
#define CAM_LOCATION_SERVICE_LOCK_CHAR_UUID 0xDD30
static esp_gatt_srvc_id_t CAM_LOCATION_SERVICE_LOCK_CHAR = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {
                .uuid16 = CAM_LOCATION_SERVICE_LOCK_CHAR_UUID,
            },
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

// READ, WRITE, Enable location updates characteristic: 0x00 - Off, 0x01 - On
#define CAM_LOCATION_SERVICE_UPDATE_CHAR_UUID 0xDD31
static esp_gatt_srvc_id_t CAM_LOCATION_SERVICE_UPDATE_CHAR = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {
                .uuid16 = CAM_LOCATION_SERVICE_UPDATE_CHAR_UUID,
            },
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

//
// Remote Service
//
static esp_bt_uuid_t CAM_REMOTE_SERVICE = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x80}}};

// NOTIFY
#define CAM_REMOTE_SERVICE_NOTIFY_CHAR_UUID 0xFF02
static esp_gatt_srvc_id_t CAM_REMOTE_SERVICE_NOTIFY_CHAR = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {
                .uuid16 = CAM_REMOTE_SERVICE_NOTIFY_CHAR_UUID,
            },
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

static const uint8_t CAM_REMOTE_FOCUS_ACQUIRED[] = {0x02, 0x3F, 0x20};
static const uint8_t CAM_REMOTE_FOCUS_LOST[] = {0x02, 0x3F, 0x00};
static const uint8_t CAM_REMOTE_SHUTTER_READY[] = {0x02, 0xA0, 0x00};
static const uint8_t CAM_REMOTE_SHUTTER_ACTIVE[] = {0x02, 0xA0, 0x20};

// Endian helpers
static inline void putBE16(uint8_t *p, uint16_t v)
{
  p[0] = (v >> 8) & 0xFF;
  p[1] = v & 0xFF;
}

static inline void putBE32(uint8_t *p, uint32_t v)
{
  p[0] = (v >> 24) & 0xFF;
  p[1] = (v >> 16) & 0xFF;
  p[2] = (v >> 8) & 0xFF;
  p[3] = v & 0xFF;
}
