#include "ble_config_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"

#include "ble_client.h"
#include "gps.h"

static const char *TAG = "ble_cfg_srv";

typedef enum {
  FIELD_CAM_NAME = 1,
  FIELD_CAM_MAC,
  FIELD_TZ_OFF,
  FIELD_DST_OFF,
  FIELD_WIFI_SSID,
  FIELD_WIFI_PASS,
  FIELD_AP_SSID,
  FIELD_AP_PASS,
  FIELD_MAX_GPS_AGE,
  FIELD_STATUS_GPS_LOCK,
  FIELD_STATUS_GPS_SATS,
  FIELD_STATUS_GPS_CONST,
  FIELD_STATUS_CAM_CONN,
  FIELD_STATUS_CAM_BOND,
} field_id_t;

static app_config_t *s_cfg;
static bool s_synced;
static bool s_adv_requested;

static const ble_uuid128_t k_svc_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x01, 0x00, 0x7E, 0xA1);

static const ble_uuid128_t k_chr_cam_name_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x02, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_cam_mac_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x03, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_tz_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x04, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_dst_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x05, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_wifi_ssid_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x07, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_wifi_pass_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x08, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_ap_ssid_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x09, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_ap_pass_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x0A, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_max_gps_age_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x0B, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_status_gps_lock_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x0C, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_status_gps_sats_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x0D, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_status_gps_const_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x0E, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_status_cam_conn_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x0F, 0x00, 0x7E, 0xA1);
static const ble_uuid128_t k_chr_status_cam_bond_uuid =
    BLE_UUID128_INIT(0xB1, 0xF0, 0xB4, 0xD5, 0x79, 0x7B, 0x5A, 0x9E,
                     0x5B, 0x4F, 0x4A, 0x1F, 0x10, 0x00, 0x7E, 0xA1);

static void copy_str_field(char *dst, size_t dst_len, const char *src, size_t src_len) {
  size_t copy_len = src_len < dst_len - 1 ? src_len : dst_len - 1;
  memcpy(dst, src, copy_len);
  dst[copy_len] = '\0';
}

static bool parse_i16_field(const char *buf, int16_t min, int16_t max,
                            int16_t *out) {
  if (buf == NULL || buf[0] == '\0') {
    return false;
  }
  char *end = NULL;
  long val = strtol(buf, &end, 10);
  if (end == buf || *end != '\0' || val < min || val > max) {
    return false;
  }
  *out = (int16_t)val;
  return true;
}

static bool parse_u32_field(const char *buf, uint32_t max, uint32_t *out) {
  if (buf == NULL || buf[0] == '\0') {
    return false;
  }
  char *end = NULL;
  unsigned long val = strtoul(buf, &end, 10);
  if (end == buf || *end != '\0' || val > max) {
    return false;
  }
  *out = (uint32_t)val;
  return true;
}

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {
  (void)conn_handle;
  (void)attr_handle;
  field_id_t field = (field_id_t)(intptr_t)arg;
  char buf[CONFIG_STR_MAX_64] = {0};
  gps_status_t status;
  app_config_t cfg;
  if (!config_get_snapshot(s_cfg, &cfg)) {
    cfg = *s_cfg;
    config_validate(&cfg);
  }

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    const char *value = "";
    char num_buf[16];
    switch (field) {
      case FIELD_CAM_NAME:
        value = cfg.camera_name_prefix;
        break;
      case FIELD_CAM_MAC:
        value = cfg.camera_mac_prefix;
        break;
      case FIELD_TZ_OFF:
        snprintf(num_buf, sizeof(num_buf), "%d", (int)cfg.tz_offset_min);
        value = num_buf;
        break;
      case FIELD_DST_OFF:
        snprintf(num_buf, sizeof(num_buf), "%d", (int)cfg.dst_offset_min);
        value = num_buf;
        break;
      case FIELD_WIFI_SSID:
        value = cfg.wifi_ssid;
        break;
      case FIELD_WIFI_PASS:
        value = cfg.wifi_pass;
        break;
      case FIELD_AP_SSID:
        value = cfg.ap_ssid;
        break;
      case FIELD_AP_PASS:
        value = cfg.ap_pass;
        break;
      case FIELD_MAX_GPS_AGE:
        snprintf(num_buf, sizeof(num_buf), "%u", (unsigned)cfg.max_gps_age_s);
        value = num_buf;
        break;
      case FIELD_STATUS_GPS_LOCK:
        if (gps_get_status(&status)) {
          snprintf(num_buf, sizeof(num_buf), "%u",
                   (unsigned)(status.has_lock ? 1 : 0));
          value = num_buf;
        }
        break;
      case FIELD_STATUS_GPS_SATS:
        if (gps_get_status(&status)) {
          snprintf(num_buf, sizeof(num_buf), "%u",
                   (unsigned)status.satellites);
          value = num_buf;
        }
        break;
      case FIELD_STATUS_GPS_CONST:
        if (gps_get_status(&status)) {
          snprintf(num_buf, sizeof(num_buf), "%u", (unsigned)status.constellations);
          value = num_buf;
        }
        break;
      case FIELD_STATUS_CAM_CONN:
        snprintf(num_buf, sizeof(num_buf), "%u",
                 (unsigned)(ble_client_is_connected() ? 1 : 0));
        value = num_buf;
        break;
      case FIELD_STATUS_CAM_BOND:
        snprintf(num_buf, sizeof(num_buf), "%u",
                 (unsigned)(ble_client_is_bonded() ? 1 : 0));
        value = num_buf;
        break;
      default:
        return BLE_ATT_ERR_UNLIKELY;
    }
    return os_mbuf_append(ctxt->om, value, strlen(value)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, NULL);
    if (rc != 0) {
      return BLE_ATT_ERR_UNLIKELY;
    }

    switch (field) {
      case FIELD_CAM_NAME:
        copy_str_field(cfg.camera_name_prefix, sizeof(cfg.camera_name_prefix), buf, strlen(buf));
        break;
      case FIELD_CAM_MAC:
        copy_str_field(cfg.camera_mac_prefix, sizeof(cfg.camera_mac_prefix), buf, strlen(buf));
        break;
      case FIELD_TZ_OFF: {
        int16_t tz;
        if (!parse_i16_field(buf, -1440, 1440, &tz)) {
          return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        cfg.tz_offset_min = tz;
        break;
      }
      case FIELD_DST_OFF: {
        int16_t dst;
        if (!parse_i16_field(buf, -1440, 1440, &dst)) {
          return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        cfg.dst_offset_min = dst;
        break;
      }
      case FIELD_WIFI_SSID:
        copy_str_field(cfg.wifi_ssid, sizeof(cfg.wifi_ssid), buf, strlen(buf));
        break;
      case FIELD_WIFI_PASS:
        copy_str_field(cfg.wifi_pass, sizeof(cfg.wifi_pass), buf, strlen(buf));
        break;
      case FIELD_AP_SSID:
        copy_str_field(cfg.ap_ssid, sizeof(cfg.ap_ssid), buf, strlen(buf));
        break;
      case FIELD_AP_PASS:
        copy_str_field(cfg.ap_pass, sizeof(cfg.ap_pass), buf, strlen(buf));
        break;
      case FIELD_MAX_GPS_AGE: {
        uint32_t max_age = 0;
        if (!parse_u32_field(buf, 86400, &max_age)) {
          return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        cfg.max_gps_age_s = max_age;
        break;
      }
      case FIELD_STATUS_GPS_LOCK:
      case FIELD_STATUS_GPS_SATS:
      case FIELD_STATUS_GPS_CONST:
      case FIELD_STATUS_CAM_CONN:
      case FIELD_STATUS_CAM_BOND:
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
      default:
        return BLE_ATT_ERR_UNLIKELY;
    }

    return config_apply(s_cfg, &cfg) ? 0 : BLE_ATT_ERR_UNLIKELY;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &k_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = &k_chr_cam_name_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_CAM_NAME},
            {.uuid = &k_chr_cam_mac_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_CAM_MAC},
            {.uuid = &k_chr_tz_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_TZ_OFF},
            {.uuid = &k_chr_dst_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_DST_OFF},
            {.uuid = &k_chr_wifi_ssid_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_WIFI_SSID},
            {.uuid = &k_chr_wifi_pass_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_WIFI_PASS},
            {.uuid = &k_chr_ap_ssid_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_AP_SSID},
            {.uuid = &k_chr_ap_pass_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_AP_PASS},
            {.uuid = &k_chr_max_gps_age_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
             .arg = (void *)FIELD_MAX_GPS_AGE},
            {.uuid = &k_chr_status_gps_lock_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ,
             .arg = (void *)FIELD_STATUS_GPS_LOCK},
            {.uuid = &k_chr_status_gps_sats_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ,
             .arg = (void *)FIELD_STATUS_GPS_SATS},
            {.uuid = &k_chr_status_gps_const_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ,
             .arg = (void *)FIELD_STATUS_GPS_CONST},
            {.uuid = &k_chr_status_cam_conn_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ,
             .arg = (void *)FIELD_STATUS_CAM_CONN},
            {.uuid = &k_chr_status_cam_bond_uuid.u,
             .access_cb = gatt_access_cb,
             .flags = BLE_GATT_CHR_F_READ,
             .arg = (void *)FIELD_STATUS_CAM_BOND},
            {0},
        },
    },
    {0},
};

void ble_config_server_register(app_config_t *cfg) {
  s_cfg = cfg;
  ble_gatts_count_cfg(gatt_svcs);
  ble_gatts_add_svcs(gatt_svcs);
}

void ble_config_server_on_sync(void) {
  s_synced = true;
  if (s_adv_requested) {
    ble_config_server_start();
  }
}

void ble_config_server_start(void) {
  s_adv_requested = true;
  if (!s_synced) {
    return;
  }

  struct ble_hs_adv_fields fields = {0};
  ble_uuid128_t svc_uuid = k_svc_uuid;
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  const char *name = ble_svc_gap_device_name();
  fields.name = (const uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;
  fields.uuids128 = &svc_uuid;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGW(TAG, "Adv fields set failed: %d", rc);
    return;
  }

  struct ble_gap_adv_params adv_params = {0};
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  uint8_t addr_type;
  rc = ble_hs_id_infer_auto(0, &addr_type);
  if (rc != 0) {
    ESP_LOGW(TAG, "Address type infer failed: %d", rc);
    return;
  }
  rc = ble_gap_adv_start(addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_client_gap_event_cb, NULL);
  if (rc != 0) {
    ESP_LOGW(TAG, "Adv start failed: %d", rc);
  } else {
    ESP_LOGI(TAG, "BLE config advertising");
  }
}

void ble_config_server_stop(void) {
  s_adv_requested = false;
  if (s_synced) {
    int rc = ble_gap_adv_stop();
    if (rc != 0) {
      ESP_LOGW(TAG, "Adv stop failed: %d", rc);
    }
  }
  ESP_LOGI(TAG, "BLE config stopped");
}
