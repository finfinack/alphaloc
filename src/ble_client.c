#include "ble_client.h"

#include <math.h>
#include <string.h>

#include "ble_config_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

static const char *TAG = "ble_client";

#ifndef ALPHALOC_VERBOSE
#define ALPHALOC_VERBOSE 0
#endif

#if ALPHALOC_VERBOSE
#define VLOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define VLOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
#define VLOGI(...) ((void)0)
#define VLOGW(...) ((void)0)
#endif

extern void ble_store_config_init(void);
extern int ble_store_util_delete_peer(const ble_addr_t *addr);

typedef struct {
  uint16_t conn_handle;
  uint16_t loc_svc_start;
  uint16_t loc_svc_end;
  uint16_t rem_svc_start;
  uint16_t rem_svc_end;
  uint16_t chr_dd11;
  uint16_t chr_dd21;
  uint16_t chr_dd30;
  uint16_t chr_dd31;
  uint16_t chr_ff02;
  uint16_t end_ff02;
  uint16_t cccd_ff02;
} ble_handles_t;

typedef enum {
  DISC_NONE = 0,
  DISC_LOC_SVC,
  DISC_ALL_SVC,
  DISC_LOC_CHR,
  DISC_ALL_CHR,
  DISC_REM_SVC,
  DISC_REM_CHR,
  DISC_REM_DSC,
} disc_state_t;

static const uint8_t SONY_MFG_PREFIX[4] = {0x2D, 0x01, 0x00, 0x03};

static ble_handles_t s_handles;
static disc_state_t s_disc_state;
static uint8_t s_own_addr_type;
static const app_config_t *s_cfg;
static ble_focus_cb_t s_focus_cb;
static void *s_focus_ctx;
static bool s_require_tz_dst;
static uint16_t s_tz_off_min;
static uint16_t s_dst_off_min;
static bool s_connecting_camera;
static uint8_t s_dd21_retry;
static bool s_location_enabled;
static bool s_dd21_ready;
static bool s_dd21_pending;
static bool s_encrypted;
static bool s_notify_pending;
static bool s_remote_disc_started;
static bool s_dsc_pending_ff02;
static bool s_dsc_in_progress;
static int64_t s_last_loc_enable_attempt_us;
static int64_t s_last_dd21_attempt_us;
typedef enum {
  DSC_NONE = 0,
  DSC_FF02,
} dsc_target_t;
static dsc_target_t s_dsc_target;
static uint8_t s_dsc_retry_count;
static esp_timer_handle_t s_dsc_retry_timer;
static int8_t s_last_chr_interest;
static bool s_ff02_cccd_deferred;
static bool s_ff02_cccd_sent;
#if ALPHALOC_VERBOSE
static bool s_payload_logged;
#endif
static bool s_bonded_camera;
static bool s_retried_disc_after_enc;

static void ble_start_scan(void);
static void schedule_dsc_retry(void);
static void try_start_ff02_dsc(uint16_t conn_handle);
static void try_start_pending_dsc(uint16_t conn_handle);
static void enable_location_updates(uint16_t conn_handle);
static bool enc_failure_needs_rebond(int status);
static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg);
static int cccd_read_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg);
typedef struct {
  const char *label;
  uint16_t handle;
  uint16_t chr_handle;
} cccd_ctx_t;

static cccd_ctx_t s_cccd_ff02_ctx = {"FF02", 0, 0};
static bool peer_is_bonded(const ble_addr_t *addr) {
  if (!addr) {
    return false;
  }
  ble_addr_t peers[MYNEWT_VAL(BLE_STORE_MAX_BONDS)];
  int num = 0;
  if (ble_store_util_bonded_peers(peers, &num,
                                  MYNEWT_VAL(BLE_STORE_MAX_BONDS)) != 0) {
    return false;
  }
  for (int i = 0; i < num; ++i) {
    if (memcmp(peers[i].val, addr->val, sizeof(addr->val)) == 0 &&
        peers[i].type == addr->type) {
      return true;
    }
  }
  return false;
}

static bool enc_failure_needs_rebond(int status) {
  if (status == 0) {
    return false;
  }
#ifdef BLE_HS_ERR_HCI_BASE
#ifdef BLE_ERR_PINKEY_MISSING
  if (status == (BLE_HS_ERR_HCI_BASE + BLE_ERR_PINKEY_MISSING)) {
    return true;
  }
#endif
#ifdef BLE_ERR_KEY_MISSING
  if (status == (BLE_HS_ERR_HCI_BASE + BLE_ERR_KEY_MISSING)) {
    return true;
  }
#endif
#ifdef BLE_ERR_AUTH_FAIL
  if (status == (BLE_HS_ERR_HCI_BASE + BLE_ERR_AUTH_FAIL)) {
    return true;
  }
#endif
#endif
#ifdef BLE_HS_ERR_SM_AUTHFAIL
  if (status == BLE_HS_ERR_SM_AUTHFAIL) {
    return true;
  }
#endif
#ifdef BLE_HS_ERR_SM_KEY_MISSING
  if (status == BLE_HS_ERR_SM_KEY_MISSING) {
    return true;
  }
#endif
  return false;
}

static void addr_to_str(const ble_addr_t *addr, char *out, size_t len) {
  snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X", addr->val[5],
           addr->val[4], addr->val[3], addr->val[2], addr->val[1],
           addr->val[0]);
}

static bool str_prefix_match(const char *value, const char *prefix) {
  if (prefix == NULL || prefix[0] == '\0') {
    return true;
  }
  if (value == NULL) {
    return false;
  }
  size_t prefix_len = strlen(prefix);
  return strncasecmp(value, prefix, prefix_len) == 0;
}

static bool name_prefix_match(const struct ble_hs_adv_fields *fields,
                              const char *prefix) {
  if (prefix == NULL || prefix[0] == '\0') {
    return true;
  }
  if (fields->name == NULL || fields->name_len == 0) {
    return false;
  }
  size_t prefix_len = strlen(prefix);
  if (fields->name_len < prefix_len) {
    return false;
  }
  return strncasecmp((const char *)fields->name, prefix, prefix_len) == 0;
}

static void log_mfg_data(const uint8_t *data, uint8_t len) {
  if (!ALPHALOC_VERBOSE) {
    return;
  }
  char buf[64];
  size_t pos = 0;
  for (uint8_t i = 0; i < len && pos + 3 < sizeof(buf); ++i) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X", data[i]);
  }
  buf[pos] = '\0';
  VLOGI("Sony ADV mfg_data len=%u data=%s", (unsigned)len, buf);
}

static void make_sony_uuid(uint16_t first, uint16_t second,
                           ble_uuid128_t *out) {
  uint8_t bytes[16] = {0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       0xFF,
                       (uint8_t)(second & 0xFF),
                       (uint8_t)(second >> 8),
                       (uint8_t)(first & 0xFF),
                       (uint8_t)(first >> 8),
                       0x00,
                       0x80};
  memcpy(out->value, bytes, sizeof(bytes));
}

static bool is_sony_camera_adv(const struct ble_gap_disc_desc *desc,
                               const struct ble_hs_adv_fields *fields) {
  if (fields->mfg_data_len < sizeof(SONY_MFG_PREFIX)) {
    return false;
  }
  uint16_t company_id =
      (uint16_t)fields->mfg_data[1] << 8 | fields->mfg_data[0];
  uint16_t product_id =
      (uint16_t)fields->mfg_data[3] << 8 | fields->mfg_data[2];
  if (company_id != 0x012D || product_id != 0x0003) {
    return false;
  }
  log_mfg_data(fields->mfg_data, fields->mfg_data_len);

  char addr_str[18];
  addr_to_str(&desc->addr, addr_str, sizeof(addr_str));

  if (!str_prefix_match(addr_str, s_cfg->camera_mac_prefix)) {
    VLOGI("ADV ignored: mac %s != %s", addr_str, s_cfg->camera_mac_prefix);
    return false;
  }

  if (s_cfg->camera_name_prefix[0] != '\0') {
    if (!name_prefix_match(fields, s_cfg->camera_name_prefix)) {
      if (fields->name == NULL || fields->name_len == 0) {
      } else {
        char name_buf[32];
        size_t copy_len = fields->name_len < sizeof(name_buf) - 1
                              ? fields->name_len
                              : sizeof(name_buf) - 1;
        memcpy(name_buf, fields->name, copy_len);
        name_buf[copy_len] = '\0';
        return false;
      }
    }
  }

  VLOGI("ADV matched Sony camera: %s", addr_str);
  return true;
}

static int gatt_disc_chrs_cb(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg);
static int gatt_disc_svc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *svc, void *arg);
static int gatt_disc_dsc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t chr_val_handle,
                            const struct ble_gatt_dsc *dsc, void *arg);
static int gatt_disc_all_svc_cb(uint16_t conn_handle,
                                const struct ble_gatt_error *error,
                                const struct ble_gatt_svc *svc, void *arg);
static bool uuid_matches(const ble_uuid_any_t *uuid, uint16_t first,
                         uint16_t second);
static bool uuid16_matches(const ble_uuid_any_t *uuid, uint16_t short_uuid);
static void start_all_char_discovery(uint16_t conn_handle);

static void start_location_service_discovery(uint16_t conn_handle) {
  ble_uuid128_t svc_uuid;
  make_sony_uuid(0xDD00, 0xDD00, &svc_uuid);
  s_disc_state = DISC_LOC_SVC;
  ble_gattc_disc_svc_by_uuid(conn_handle, &svc_uuid.u, gatt_disc_svc_cb, NULL);
}

static void start_remote_service_discovery(uint16_t conn_handle) {
  ble_uuid128_t svc_uuid;
  make_sony_uuid(0xFF00, 0xFF00, &svc_uuid);
  s_disc_state = DISC_REM_SVC;
  VLOGI("Starting remote service discovery");
  ble_gattc_disc_svc_by_uuid(conn_handle, &svc_uuid.u, gatt_disc_svc_cb, NULL);
}

static void start_all_char_discovery(uint16_t conn_handle) {
  s_disc_state = DISC_ALL_CHR;
  ble_gattc_disc_all_chrs(conn_handle, 1, 0xFFFF, gatt_disc_chrs_cb, NULL);
}

static int dd21_read_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg) {
  (void)conn_handle;
  (void)arg;
  if (error->status != 0 || attr == NULL || attr->om == NULL) {
    ESP_LOGW(TAG, "DD21 read failed: %d", error->status);
    if (s_dd21_retry < 2 && s_handles.chr_dd21 != 0) {
      s_dd21_retry++;
      ble_gattc_read(conn_handle, s_handles.chr_dd21, dd21_read_cb, NULL);
    }
    return 0;
  }

  uint8_t buf[7] = {0};
  int rc = ble_hs_mbuf_to_flat(attr->om, buf, sizeof(buf), NULL);
  if (rc != 0) {
    ESP_LOGW(TAG, "DD21 read decode failed: %d", rc);
    return 0;
  }
  s_require_tz_dst = (buf[4] & 0x02) != 0;
  s_dd21_ready = true;
  ESP_LOGI(TAG, "DD21 flag byte=0x%02X require_tz_dst=%d", buf[4],
           s_require_tz_dst);
  enable_location_updates(conn_handle);
  return 0;
}

static void enable_notifications(uint16_t conn_handle) {
  if (s_handles.cccd_ff02 == 0) {
    ESP_LOGW(TAG, "CCCD handle not found");
    return;
  }
  if (!s_encrypted) {
    s_notify_pending = true;
    VLOGI("Deferring notifications until encryption");
    return;
  }
  s_cccd_ff02_ctx.handle = s_handles.cccd_ff02;
  s_cccd_ff02_ctx.chr_handle = s_handles.chr_ff02;
  uint8_t val_le[2] = {0x01, 0x00};
  ble_gattc_write_flat(conn_handle, s_handles.cccd_ff02, val_le, sizeof(val_le),
                       cccd_write_cb, &s_cccd_ff02_ctx);
  ESP_LOGI(TAG, "Subscribing to FF02 notifications");
  s_notify_pending = false;
}

static int cccd_write_cb(uint16_t conn_handle,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg) {
  (void)attr;
  cccd_ctx_t *ctx = (cccd_ctx_t *)arg;
  const char *label = ctx ? ctx->label : "CCCD";
  if (error->status != 0) {
    ESP_LOGW(TAG, "%s CCCD write failed: %d", label, error->status);
    return 0;
  }
  ESP_LOGI(TAG, "%s CCCD write ok", label);
  if (ctx) {
    ble_gattc_read(conn_handle, ctx->handle, cccd_read_cb, ctx);
  }
  return 0;
}

static int cccd_read_cb(uint16_t conn_handle,
                        const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg) {
  (void)conn_handle;
  cccd_ctx_t *ctx = (cccd_ctx_t *)arg;
  const char *label = ctx ? ctx->label : "CCCD";
  if (error->status != 0 || attr == NULL || attr->om == NULL) {
    ESP_LOGW(TAG, "%s CCCD read failed: %d", label, error->status);
    return 0;
  }
  uint8_t buf[2] = {0};
  int rc = ble_hs_mbuf_to_flat(attr->om, buf, sizeof(buf), NULL);
  if (rc != 0) {
    ESP_LOGW(TAG, "%s CCCD read decode failed: %d", label, rc);
    return 0;
  }
  uint16_t val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  ESP_LOGI(TAG, "%s CCCD value=0x%04X", label, val);
  return 0;
}

static void try_start_ff02_dsc(uint16_t conn_handle) {
  if (s_handles.chr_ff02 == 0 || s_handles.cccd_ff02 != 0) {
    return;
  }
  if (s_dsc_in_progress) {
    s_dsc_pending_ff02 = true;
    return;
  }
  if (!s_encrypted) {
    s_dsc_pending_ff02 = true;
    schedule_dsc_retry();
    return;
  }
  s_disc_state = DISC_REM_DSC;
  s_dsc_in_progress = true;
  s_dsc_target = DSC_FF02;
  uint16_t end = s_handles.end_ff02
                     ? s_handles.end_ff02
                     : (s_handles.rem_svc_end ? s_handles.rem_svc_end : 0xFFFF);
  int rc = ble_gattc_disc_all_dscs(conn_handle, s_handles.chr_ff02, end,
                                   gatt_disc_dsc_cb, NULL);
  if (rc == 0) {
    s_dsc_pending_ff02 = false;
    return;
  }
  s_dsc_in_progress = false;
  s_dsc_pending_ff02 = true;
  VLOGW("Descriptor discovery start failed: %d", rc);
  schedule_dsc_retry();
}

static void try_start_pending_dsc(uint16_t conn_handle) {
  if (s_dsc_in_progress) {
    return;
  }
  if (s_dsc_pending_ff02) {
    try_start_ff02_dsc(conn_handle);
  }
}

static void dsc_retry_cb(void *arg) {
  (void)arg;
  if (s_handles.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }
  if (s_dsc_retry_count >= 4) {
    if (s_dsc_pending_ff02 && s_handles.chr_ff02 != 0 &&
        s_handles.cccd_ff02 == 0) {
      s_dsc_pending_ff02 = false;
      s_handles.cccd_ff02 = (uint16_t)(s_handles.chr_ff02 + 1);
      ESP_LOGW(TAG, "FF02 CCCD not found; using fallback handle=%u",
               s_handles.cccd_ff02);
      enable_notifications(s_handles.conn_handle);
    }
    return;
  }
  s_dsc_retry_count++;
  try_start_pending_dsc(s_handles.conn_handle);
}

static void schedule_dsc_retry(void) {
  if (!s_dsc_retry_timer) {
    return;
  }
  esp_timer_stop(s_dsc_retry_timer);
  esp_timer_start_once(s_dsc_retry_timer, 500000);
}

static void enable_location_updates(uint16_t conn_handle) {
  if (s_handles.chr_dd21 != 0) {
    if (!s_encrypted) {
      s_dd21_pending = true;
      VLOGI("Deferring DD21 read until encryption");
    } else if (!s_dd21_ready) {
      s_dd21_pending = false;
      ble_gattc_read(conn_handle, s_handles.chr_dd21, dd21_read_cb, NULL);
    }
  }
  if (!s_encrypted || !s_dd21_ready) {
    VLOGI("Deferring DD30/DD31 writes until encrypted and DD21 read");
    return;
  }
  if (s_handles.chr_dd30 != 0) {
    uint8_t on = 0x01;
    ESP_LOGI(TAG, "Unlocking location");
    ble_gattc_write_flat(conn_handle, s_handles.chr_dd30, &on, sizeof(on), NULL,
                         NULL);
  }
  if (s_handles.chr_dd31 != 0) {
    uint8_t on = 0x01;
    ESP_LOGI(TAG, "Enabling location updates");
    ble_gattc_write_flat(conn_handle, s_handles.chr_dd31, &on, sizeof(on), NULL,
                         NULL);
  }
  s_location_enabled = (s_handles.chr_dd30 != 0 && s_handles.chr_dd31 != 0);
  ESP_LOGI(TAG, "Location updates enabled");
}

static int gatt_disc_svc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *svc, void *arg) {
  if (error->status == 0 && svc != NULL) {
    if (s_disc_state == DISC_LOC_SVC) {
      if (s_handles.loc_svc_start == 0) {
        s_handles.loc_svc_start = svc->start_handle;
        s_handles.loc_svc_end = svc->end_handle;
        VLOGI("Location service found: start=%u end=%u", svc->start_handle,
              svc->end_handle);
      }
    } else if (s_disc_state == DISC_REM_SVC) {
      if (s_handles.rem_svc_start == 0) {
        s_handles.rem_svc_start = svc->start_handle;
        s_handles.rem_svc_end = svc->end_handle;
        VLOGI("Remote service found: start=%u end=%u", svc->start_handle,
              svc->end_handle);
      }
    }
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    if (s_disc_state == DISC_LOC_SVC && s_handles.loc_svc_start != 0) {
      s_disc_state = DISC_LOC_CHR;
      return ble_gattc_disc_all_chrs(conn_handle, s_handles.loc_svc_start,
                                     s_handles.loc_svc_end, gatt_disc_chrs_cb,
                                     NULL);
    }
    if (s_disc_state == DISC_LOC_SVC && s_handles.loc_svc_start == 0) {
      VLOGI("Location service not found by UUID, falling back to all services");
      s_disc_state = DISC_ALL_SVC;
      return ble_gattc_disc_all_svcs(conn_handle, gatt_disc_all_svc_cb, NULL);
    }
    if (s_disc_state == DISC_REM_SVC && s_handles.rem_svc_start != 0) {
      s_disc_state = DISC_REM_CHR;
      return ble_gattc_disc_all_chrs(conn_handle, s_handles.rem_svc_start,
                                     s_handles.rem_svc_end, gatt_disc_chrs_cb,
                                     NULL);
    }
    if (s_disc_state == DISC_REM_SVC && s_handles.rem_svc_start == 0) {
      VLOGI("Remote service not found by UUID, falling back to all services");
      s_disc_state = DISC_ALL_SVC;
      return ble_gattc_disc_all_svcs(conn_handle, gatt_disc_all_svc_cb, NULL);
    }
  }
  if (error->status != BLE_HS_EDONE) {
    VLOGW("Service discovery error=%d state=%d", error->status, s_disc_state);
  }
  return error->status;
}

static int gatt_disc_all_svc_cb(uint16_t conn_handle,
                                const struct ble_gatt_error *error,
                                const struct ble_gatt_svc *svc, void *arg) {
  if (error->status == 0 && svc != NULL) {
    if ((uuid_matches(&svc->uuid, 0xDD00, 0xDD00) ||
         uuid16_matches(&svc->uuid, 0xDD00)) &&
        s_handles.loc_svc_start == 0) {
      s_handles.loc_svc_start = svc->start_handle;
      s_handles.loc_svc_end = svc->end_handle;
      VLOGI("Location service found in all-svc: start=%u end=%u",
            svc->start_handle, svc->end_handle);
    } else if ((uuid_matches(&svc->uuid, 0xFF00, 0xFF00) ||
                uuid16_matches(&svc->uuid, 0xFF00)) &&
               s_handles.rem_svc_start == 0) {
      s_handles.rem_svc_start = svc->start_handle;
      s_handles.rem_svc_end = svc->end_handle;
      VLOGI("Remote service found in all-svc: start=%u end=%u",
            svc->start_handle, svc->end_handle);
    }
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    if (s_handles.loc_svc_start != 0 && s_handles.chr_dd11 == 0) {
      s_disc_state = DISC_LOC_CHR;
      return ble_gattc_disc_all_chrs(conn_handle, s_handles.loc_svc_start,
                                     s_handles.loc_svc_end, gatt_disc_chrs_cb,
                                     NULL);
    }
    if (s_handles.rem_svc_start != 0) {
      s_disc_state = DISC_REM_CHR;
      return ble_gattc_disc_all_chrs(conn_handle, s_handles.rem_svc_start,
                                     s_handles.rem_svc_end, gatt_disc_chrs_cb,
                                     NULL);
    }
    ESP_LOGW(TAG, "Sony services not found in all-svc scan");
  }
  return error->status;
}

static bool uuid_matches(const ble_uuid_any_t *uuid, uint16_t first,
                         uint16_t second) {
  ble_uuid128_t target;
  make_sony_uuid(first, second, &target);
  return ble_uuid_cmp(&uuid->u, &target.u) == 0;
}

static bool uuid16_matches(const ble_uuid_any_t *uuid, uint16_t short_uuid) {
  return uuid->u.type == BLE_UUID_TYPE_16 && uuid->u16.value == short_uuid;
}

static int gatt_disc_chrs_cb(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg) {
  if (error->status == 0 && chr != NULL) {
    if (s_last_chr_interest == 1 && s_handles.end_ff02 == 0 &&
        chr->def_handle > 0) {
      s_handles.end_ff02 = (uint16_t)(chr->def_handle - 1);
      s_last_chr_interest = 0;
    }
    if (s_disc_state == DISC_LOC_CHR || s_disc_state == DISC_ALL_CHR) {
      if (uuid_matches(&chr->uuid, 0xDD11, 0xDD00) ||
          uuid16_matches(&chr->uuid, 0xDD11)) {
        s_handles.chr_dd11 = chr->val_handle;
        VLOGI("Found DD11 handle=%u", chr->val_handle);
      } else if (uuid_matches(&chr->uuid, 0xDD21, 0xDD00) ||
                 uuid16_matches(&chr->uuid, 0xDD21)) {
        s_handles.chr_dd21 = chr->val_handle;
        VLOGI("Found DD21 handle=%u", chr->val_handle);
      } else if (uuid_matches(&chr->uuid, 0xDD30, 0xDD00) ||
                 uuid16_matches(&chr->uuid, 0xDD30)) {
        s_handles.chr_dd30 = chr->val_handle;
        VLOGI("Found DD30 handle=%u", chr->val_handle);
      } else if (uuid_matches(&chr->uuid, 0xDD31, 0xDD00) ||
                 uuid16_matches(&chr->uuid, 0xDD31)) {
        s_handles.chr_dd31 = chr->val_handle;
        VLOGI("Found DD31 handle=%u", chr->val_handle);
      }
    } else if (s_disc_state == DISC_REM_CHR) {
      if (uuid_matches(&chr->uuid, 0xFF02, 0xFF00) ||
          uuid16_matches(&chr->uuid, 0xFF02)) {
        s_handles.chr_ff02 = chr->val_handle;
        VLOGI("Found FF02 handle=%u props=0x%02X (remote svc)", chr->val_handle,
              chr->properties);
        s_last_chr_interest = 1;
      }
    }
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    if (s_last_chr_interest == 1 && s_handles.end_ff02 == 0) {
      s_handles.end_ff02 =
          s_handles.rem_svc_end ? s_handles.rem_svc_end : 0xFFFF;
    }
    s_last_chr_interest = 0;
    if (s_disc_state == DISC_LOC_CHR) {
      if (s_handles.chr_dd11 == 0) {
        VLOGI("DD11 not found in service range, scanning all characteristics");
        start_all_char_discovery(conn_handle);
        return 0;
      }
      enable_location_updates(conn_handle);
      if (s_handles.rem_svc_start != 0) {
        s_disc_state = DISC_REM_CHR;
        return ble_gattc_disc_all_chrs(conn_handle, s_handles.rem_svc_start,
                                       s_handles.rem_svc_end, gatt_disc_chrs_cb,
                                       NULL);
      }
      if (!s_remote_disc_started) {
        s_remote_disc_started = true;
        start_remote_service_discovery(conn_handle);
      }
      return 0;
    }
    if (s_disc_state == DISC_ALL_CHR) {
      if (s_handles.chr_dd11 == 0) {
        ESP_LOGW(TAG, "DD11 not found in full characteristic scan");
      } else {
        enable_location_updates(conn_handle);
      }
      if (s_handles.rem_svc_start != 0) {
        s_disc_state = DISC_REM_CHR;
        return ble_gattc_disc_all_chrs(conn_handle, s_handles.rem_svc_start,
                                       s_handles.rem_svc_end, gatt_disc_chrs_cb,
                                       NULL);
      }
      if (!s_remote_disc_started) {
        s_remote_disc_started = true;
        start_remote_service_discovery(conn_handle);
      }
      return 0;
    }
    if (s_disc_state == DISC_REM_CHR) {
      if (s_handles.chr_ff02 != 0) {
        s_disc_state = DISC_REM_DSC;
        return ble_gattc_disc_all_dscs(conn_handle, s_handles.rem_svc_start,
                                       s_handles.rem_svc_end, gatt_disc_dsc_cb,
                                       NULL);
      }
      ESP_LOGW(TAG, "FF02 not found in remote service");
    }
  }
  if (error->status != BLE_HS_EDONE) {
    VLOGW("Characteristic discovery error=%d state=%d", error->status,
          s_disc_state);
  }
  return error->status;
}

static int gatt_disc_dsc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t chr_val_handle,
                            const struct ble_gatt_dsc *dsc, void *arg) {
  if (error->status == 0 && dsc != NULL) {
    if (dsc->uuid.u.type == BLE_UUID_TYPE_16 &&
        dsc->uuid.u16.value == BLE_GATT_DSC_CLT_CFG_UUID16) {
      if (chr_val_handle == s_handles.chr_ff02) {
        s_handles.cccd_ff02 = dsc->handle;
        VLOGI("Found FF02 CCCD handle=%u", dsc->handle);
      }
    }
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    if (s_dsc_target == DSC_FF02) {
      if (s_handles.cccd_ff02 != 0) {
        s_ff02_cccd_deferred = true;
      } else {
        s_handles.cccd_ff02 = (uint16_t)(s_handles.chr_ff02 + 1);
        ESP_LOGW(TAG, "FF02 CCCD not found; using fallback handle=%u",
                 s_handles.cccd_ff02);
        s_ff02_cccd_deferred = true;
      }
      s_dsc_in_progress = false;
      s_dsc_target = DSC_NONE;
      try_start_pending_dsc(conn_handle);
    }
  }
  return error->status;
}

static bool build_location_payload(const gps_fix_t *fix, bool require_tz_dst,
                                   uint16_t tz_off_min, uint16_t dst_off_min,
                                   uint8_t *out, size_t *out_len) {
  if (!fix || !fix->valid) {
    return false;
  }

  const bool send_tz_dst =
      require_tz_dst || (tz_off_min > 0 || dst_off_min > 0);
  const size_t total_len = send_tz_dst ? 95 : 91;

  int32_t lat_scaled = (int32_t)llround(fix->lat_deg * 1e7);
  int32_t lon_scaled = (int32_t)llround(fix->lon_deg * 1e7);

  out[0] = (uint8_t)((send_tz_dst ? 0x5D : 0x59) >> 8);
  out[1] = (uint8_t)((send_tz_dst ? 0x5D : 0x59) & 0xFF);

  out[2] = 0x08;
  out[3] = 0x02;
  out[4] = 0xFC;
  out[5] = send_tz_dst ? 0x03 : 0x00;
  out[6] = 0x00;
  out[7] = 0x00;
  out[8] = 0x10;
  out[9] = 0x10;
  out[10] = 0x10;

  out[11] = (uint8_t)((uint32_t)lat_scaled >> 24);
  out[12] = (uint8_t)((uint32_t)lat_scaled >> 16);
  out[13] = (uint8_t)((uint32_t)lat_scaled >> 8);
  out[14] = (uint8_t)((uint32_t)lat_scaled);
  out[15] = (uint8_t)((uint32_t)lon_scaled >> 24);
  out[16] = (uint8_t)((uint32_t)lon_scaled >> 16);
  out[17] = (uint8_t)((uint32_t)lon_scaled >> 8);
  out[18] = (uint8_t)((uint32_t)lon_scaled);

  out[19] = (uint8_t)(fix->year >> 8);
  out[20] = (uint8_t)(fix->year & 0xFF);
  out[21] = fix->month;
  out[22] = fix->day;
  out[23] = fix->hour;
  out[24] = fix->minute;
  out[25] = fix->second;

  memset(out + 26, 0, (send_tz_dst ? 95 : 91) - 26);

  if (send_tz_dst) {
    out[91] = (uint8_t)(tz_off_min >> 8);
    out[92] = (uint8_t)(tz_off_min & 0xFF);
    out[93] = (uint8_t)(dst_off_min >> 8);
    out[94] = (uint8_t)(dst_off_min & 0xFF);
  }

  *out_len = total_len;
  return true;
}

bool ble_client_send_location(const gps_fix_t *fix) {
  if (s_handles.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    VLOGW("Skip location send: no connection");
    return false;
  }
  if (!s_location_enabled) {
    VLOGW("Skip location send: location updates not enabled");
    if (s_encrypted && s_dd21_ready) {
      int64_t now = esp_timer_get_time();
      if (now - s_last_loc_enable_attempt_us > 3000000) {
        s_last_loc_enable_attempt_us = now;
        enable_location_updates(s_handles.conn_handle);
      }
    } else if (s_encrypted && s_handles.chr_dd21 != 0) {
      int64_t now = esp_timer_get_time();
      if (now - s_last_dd21_attempt_us > 3000000) {
        s_last_dd21_attempt_us = now;
        ble_gattc_read(s_handles.conn_handle, s_handles.chr_dd21, dd21_read_cb,
                       NULL);
      }
    }
    return false;
  }
  if (s_handles.chr_dd21 != 0 && !s_dd21_ready) {
    VLOGW("Skip location send: DD21 not ready");
    return false;
  }
  if (s_handles.chr_dd11 == 0) {
    VLOGW("Skip location send: DD11 not discovered");
    return false;
  }
  uint8_t payload[95];
  size_t payload_len = 0;
  if (!build_location_payload(fix, s_require_tz_dst, s_tz_off_min,
                              s_dst_off_min, payload, &payload_len)) {
    ESP_LOGW(TAG, "Location payload unavailable");
    return false;
  }
  uint16_t mtu = ble_att_mtu(s_handles.conn_handle);
  uint16_t max_payload = mtu > 3 ? (uint16_t)(mtu - 3) : 0;
  int rc = 0;
  if (payload_len > max_payload) {
    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, (uint16_t)payload_len);
    if (om == NULL) {
      ESP_LOGW(TAG, "Location write failed: no mbuf");
      return false;
    }
    rc = ble_gattc_write_long(s_handles.conn_handle, s_handles.chr_dd11, 0, om,
                              NULL, NULL);
  } else {
    rc = ble_gattc_write_flat(s_handles.conn_handle, s_handles.chr_dd11,
                              payload, payload_len, NULL, NULL);
  }
#if ALPHALOC_VERBOSE
  if (!s_payload_logged) {
    s_payload_logged = true;
    ESP_LOGI(TAG, "Location payload (%u bytes):", (unsigned)payload_len);
    ESP_LOG_BUFFER_HEX(TAG, payload, payload_len);
  }
#endif
  VLOGI("Location write %s (%u bytes)", rc == 0 ? "ok" : "failed",
        (unsigned)payload_len);
  if (rc == 0 && s_ff02_cccd_deferred && !s_ff02_cccd_sent && s_encrypted &&
      s_handles.cccd_ff02 != 0) {
    s_ff02_cccd_sent = true;
    ESP_LOGI(TAG, "Enabling FF02 notifications after first location write");
    enable_notifications(s_handles.conn_handle);
  }
  return rc == 0;
}

bool ble_client_is_connected(void) {
  return s_handles.conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool ble_client_is_bonded(void) { return s_bonded_camera; }

int ble_client_gap_event_cb(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_DISC: {
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                event->disc.length_data) != 0) {
      return 0;
    }
    if (!is_sony_camera_adv(&event->disc, &fields)) {
      return 0;
    }
    ble_gap_disc_cancel();
    s_connecting_camera = true;
    ble_gap_connect(s_own_addr_type, &event->disc.addr, 30000, NULL,
                    ble_client_gap_event_cb, NULL);
    VLOGI("Connecting to Sony camera");
    return 0;
  }
  case BLE_GAP_EVENT_CONNECT: {
    if (event->connect.status == 0) {
      if (s_connecting_camera) {
        s_handles.conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected to camera");
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(s_handles.conn_handle, &desc) == 0 &&
            peer_is_bonded(&desc.peer_ota_addr)) {
          ESP_LOGI(TAG, "Existing bond found; skipping pairing");
        }
        VLOGI("Start security");
        ble_gap_security_initiate(s_handles.conn_handle);
        start_location_service_discovery(s_handles.conn_handle);
        s_connecting_camera = false;
      } else {
        ESP_LOGI(TAG, "Config client connected");
      }
    } else {
      ESP_LOGW(TAG, "Connect failed: %d", event->connect.status);
      s_connecting_camera = false;
      ble_start_scan();
    }
    return 0;
  }
  case BLE_GAP_EVENT_DISCONNECT: {
    if (event->disconnect.conn.conn_handle == s_handles.conn_handle) {
      ESP_LOGI(TAG, "Camera disconnected");
      memset(&s_handles, 0, sizeof(s_handles));
      s_handles.conn_handle = BLE_HS_CONN_HANDLE_NONE;
      s_connecting_camera = false;
      s_location_enabled = false;
      s_dd21_ready = false;
      s_dd21_pending = false;
      s_encrypted = false;
      s_notify_pending = false;
      s_remote_disc_started = false;
      s_dsc_pending_ff02 = false;
      s_dsc_in_progress = false;
      s_dsc_target = DSC_NONE;
      s_dsc_retry_count = 0;
      s_last_chr_interest = 0;
      s_ff02_cccd_deferred = false;
      s_ff02_cccd_sent = false;
      s_bonded_camera = false;
      s_retried_disc_after_enc = false;
      ble_start_scan();
    } else {
      ESP_LOGI(TAG, "Config client disconnected");
    }
    return 0;
  }
  case BLE_GAP_EVENT_NOTIFY_RX: {
#if ALPHALOC_VERBOSE
    ESP_LOGI(TAG, "Notify rx handle=%u len=%u", event->notify_rx.attr_handle,
             (unsigned)event->notify_rx.om->om_len);
    ESP_LOG_BUFFER_HEX(TAG, event->notify_rx.om->om_data,
                       event->notify_rx.om->om_len);
#endif
    if (event->notify_rx.attr_handle == s_handles.chr_ff02) {
      uint8_t focus_msg[] = {0x02, 0x3F, 0x20};
      if (event->notify_rx.om->om_len == sizeof(focus_msg) &&
          memcmp(event->notify_rx.om->om_data, focus_msg, sizeof(focus_msg)) ==
              0) {
        ESP_LOGI(TAG, "Focus acquired notification");
        if (s_focus_cb) {
          s_focus_cb(s_focus_ctx);
        }
      }
    }
    return 0;
  }
  case BLE_GAP_EVENT_PASSKEY_ACTION: {
    struct ble_sm_io pkey = {0};
    pkey.action = event->passkey.params.action;
    if (pkey.action == BLE_SM_IOACT_DISP || pkey.action == BLE_SM_IOACT_INPUT) {
      pkey.passkey = s_cfg ? s_cfg->ble_passkey : 0;
      ble_sm_inject_io(event->passkey.conn_handle, &pkey);
      ESP_LOGI(TAG, "Passkey used for pairing");
    }
    return 0;
  }
  case BLE_GAP_EVENT_REPEAT_PAIRING: {
    struct ble_gap_conn_desc desc;
    if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
      ble_store_util_delete_peer(&desc.peer_ota_addr);
      ESP_LOGW(TAG, "Repeat pairing requested; deleted existing bond");
    }
    return BLE_GAP_REPEAT_PAIRING_RETRY;
  }
  case BLE_GAP_EVENT_ENC_CHANGE:
    s_encrypted = (event->enc_change.status == 0);
    if (!s_encrypted) {
      ESP_LOGW(TAG, "Encryption failed");
    }
    if (!s_encrypted && enc_failure_needs_rebond(event->enc_change.status)) {
      struct ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
        ble_store_util_delete_peer(&desc.peer_ota_addr);
        ESP_LOGW(TAG,
                 "Bond mismatch suspected; deleting bond and retrying pairing");
      }
      ble_gap_security_initiate(event->enc_change.conn_handle);
    }
    if (s_encrypted) {
      struct ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
        s_bonded_camera = desc.sec_state.bonded;
      } else {
        s_bonded_camera = false;
      }
      if (!s_retried_disc_after_enc && s_handles.loc_svc_start == 0 &&
          s_handles.rem_svc_start == 0) {
        s_retried_disc_after_enc = true;
        s_disc_state = DISC_NONE;
        s_remote_disc_started = false;
        start_location_service_discovery(s_handles.conn_handle);
      }
    } else {
      s_bonded_camera = false;
    }
    ESP_LOGI(TAG, "Encryption %s", s_encrypted ? "enabled" : "failed");
    if (s_encrypted && s_dd21_pending && s_handles.chr_dd21 != 0 &&
        !s_dd21_ready) {
      s_dd21_pending = false;
      ble_gattc_read(s_handles.conn_handle, s_handles.chr_dd21, dd21_read_cb,
                     NULL);
    }
    if (s_encrypted && s_notify_pending && s_handles.cccd_ff02 != 0) {
      enable_notifications(s_handles.conn_handle);
    }
    return 0;
  default:
    return 0;
  }
}

static void ble_start_scan(void) {
  struct ble_gap_disc_params params = {0};
  params.passive = 0;
  // Reduce scan duty cycle for power savings (Issue 3.1)
  // Previously: 0x0010/0x0010 = 100% duty cycle
  // Now: 0x0010/0x0100 = ~10% duty cycle (saves 70-80% power)
  params.itvl = 0x0100;   // 160ms scan interval
  params.window = 0x0010; // 10ms scan window
  params.filter_duplicates = 1;
  ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params,
               ble_client_gap_event_cb, NULL);
  ESP_LOGI(TAG, "BLE scanning");
}

static void ble_on_sync(void) {
  ble_hs_id_infer_auto(0, &s_own_addr_type);
  ble_start_scan();
  ble_config_server_on_sync();
}

static void ble_host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void ble_client_init(const app_config_t *cfg) {
  s_cfg = cfg;
  memset(&s_handles, 0, sizeof(s_handles));
  s_bonded_camera = false;
  s_handles.conn_handle = BLE_HS_CONN_HANDLE_NONE;
  s_disc_state = DISC_NONE;
  s_require_tz_dst = false;
  s_tz_off_min = cfg ? cfg->tz_offset_min : 0;
  s_dst_off_min = cfg ? cfg->dst_offset_min : 0;
  s_connecting_camera = false;
  s_dd21_retry = 0;
  s_location_enabled = false;
  s_dd21_ready = false;
  s_dd21_pending = false;
  s_encrypted = false;
  s_notify_pending = false;
  s_remote_disc_started = false;
  s_dsc_pending_ff02 = false;
  s_dsc_in_progress = false;
  s_last_loc_enable_attempt_us = 0;
  s_last_dd21_attempt_us = 0;
  s_dsc_target = DSC_NONE;
  s_dsc_retry_count = 0;
  s_last_chr_interest = 0;
  s_ff02_cccd_deferred = false;
  s_ff02_cccd_sent = false;
  s_retried_disc_after_enc = false;
#if ALPHALOC_VERBOSE
  s_payload_logged = false;
#endif

  nimble_port_init();
  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_store_config_init();
  ble_config_server_register((app_config_t *)cfg);

  ble_svc_gap_device_name_set("AlphaLoc");

  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_hs_cfg.reset_cb = NULL;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist =
      BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist =
      BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  const esp_timer_create_args_t dsc_timer_args = {
      .callback = dsc_retry_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "ff02_dsc_retry",
      .skip_unhandled_events = true,
  };
  esp_timer_create(&dsc_timer_args, &s_dsc_retry_timer);

  nimble_port_freertos_init(ble_host_task);
}

void ble_client_set_focus_callback(ble_focus_cb_t cb, void *ctx) {
  s_focus_cb = cb;
  s_focus_ctx = ctx;
}

void ble_client_deinit(void) {
  // Stop and delete the retry timer to prevent resource leak
  if (s_dsc_retry_timer) {
    esp_timer_stop(s_dsc_retry_timer);
    esp_timer_delete(s_dsc_retry_timer);
    s_dsc_retry_timer = NULL;
  }
}
