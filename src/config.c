#include "config.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE "alphaloc"
#define GPS_INTERVAL_MIN_MS 1000U
#define GPS_INTERVAL_MAX_MS 60000U
#define MAX_GPS_AGE_MAX_S 86400U
#define CONFIG_WINDOW_MAX_S 3600U
#define TZ_OFFSET_MIN_MIN -1440
#define TZ_OFFSET_MAX_MIN 1440
#define BLE_PASSKEY_MAX 999999U

static const char *TAG = "config";
static SemaphoreHandle_t s_config_mutex = NULL;

static void config_ensure_mutex(void)
{
  if (s_config_mutex == NULL)
  {
    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL)
    {
      ESP_LOGE(TAG, "Failed to create config mutex");
    }
  }
}

static void config_set_str_default(char *dst, size_t dst_len, const char *src)
{
  if (!dst || dst_len == 0)
  {
    return;
  }
  if (!src)
  {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, dst_len - 1);
  dst[dst_len - 1] = '\0';
}

static void config_terminate_str(char *dst, size_t dst_len)
{
  if (dst && dst_len > 0)
  {
    dst[dst_len - 1] = '\0';
  }
}

static void config_reset_default_str(char *dst, size_t dst_len,
                                     const char *src)
{
  config_set_str_default(dst, dst_len, src);
}

void config_set_defaults(app_config_t *cfg)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->gps_interval_ms = 5000;
  cfg->max_gps_age_s = 300;
  cfg->config_window_s = 300;
  cfg->ble_passkey = 123456;
  cfg->tz_offset_min = 60;
  cfg->dst_offset_min = 60;
#ifdef ALPHALOC_DEFAULT_CAMERA_NAME_PREFIX
  config_set_str_default(cfg->camera_name_prefix,
                         sizeof(cfg->camera_name_prefix),
                         ALPHALOC_DEFAULT_CAMERA_NAME_PREFIX);
#else
  config_set_str_default(cfg->camera_name_prefix,
                         sizeof(cfg->camera_name_prefix), "SonyA7");
#endif

#ifdef ALPHALOC_DEFAULT_CAMERA_MAC_PREFIX
  config_set_str_default(cfg->camera_mac_prefix,
                         sizeof(cfg->camera_mac_prefix),
                         ALPHALOC_DEFAULT_CAMERA_MAC_PREFIX);
#else
  cfg->camera_mac_prefix[0] = '\0';
#endif

#ifdef ALPHALOC_DEFAULT_AP_SSID
  config_set_str_default(cfg->ap_ssid, sizeof(cfg->ap_ssid),
                         ALPHALOC_DEFAULT_AP_SSID);
#else
  config_set_str_default(cfg->ap_ssid, sizeof(cfg->ap_ssid), "AlphaLoc");
#endif

#ifdef ALPHALOC_DEFAULT_AP_PASS
  config_set_str_default(cfg->ap_pass, sizeof(cfg->ap_pass),
                         ALPHALOC_DEFAULT_AP_PASS);
#else
  config_set_str_default(cfg->ap_pass, sizeof(cfg->ap_pass), "alphaloc1234");
#endif

#ifdef ALPHALOC_DEFAULT_WIFI_SSID
  config_set_str_default(cfg->wifi_ssid, sizeof(cfg->wifi_ssid),
                         ALPHALOC_DEFAULT_WIFI_SSID);
#else
  config_set_str_default(cfg->wifi_ssid, sizeof(cfg->wifi_ssid), "WiFi");
#endif

#ifdef ALPHALOC_DEFAULT_WIFI_PASS
  config_set_str_default(cfg->wifi_pass, sizeof(cfg->wifi_pass),
                         ALPHALOC_DEFAULT_WIFI_PASS);
#else
  config_set_str_default(cfg->wifi_pass, sizeof(cfg->wifi_pass), "changeme");
#endif
}

void config_validate(app_config_t *cfg)
{
  if (!cfg)
  {
    return;
  }

  app_config_t defaults;
  config_set_defaults(&defaults);

  if (cfg->gps_interval_ms < GPS_INTERVAL_MIN_MS ||
      cfg->gps_interval_ms > GPS_INTERVAL_MAX_MS)
  {
    cfg->gps_interval_ms = defaults.gps_interval_ms;
  }
  if (cfg->max_gps_age_s > MAX_GPS_AGE_MAX_S)
  {
    cfg->max_gps_age_s = defaults.max_gps_age_s;
  }
  if (cfg->config_window_s > CONFIG_WINDOW_MAX_S)
  {
    cfg->config_window_s = defaults.config_window_s;
  }
  if (cfg->ble_passkey > BLE_PASSKEY_MAX)
  {
    cfg->ble_passkey = defaults.ble_passkey;
  }
  if (cfg->tz_offset_min < TZ_OFFSET_MIN_MIN ||
      cfg->tz_offset_min > TZ_OFFSET_MAX_MIN)
  {
    cfg->tz_offset_min = defaults.tz_offset_min;
  }
  if (cfg->dst_offset_min < TZ_OFFSET_MIN_MIN ||
      cfg->dst_offset_min > TZ_OFFSET_MAX_MIN)
  {
    cfg->dst_offset_min = defaults.dst_offset_min;
  }

  config_terminate_str(cfg->camera_name_prefix,
                       sizeof(cfg->camera_name_prefix));
  config_terminate_str(cfg->camera_mac_prefix, sizeof(cfg->camera_mac_prefix));
  config_terminate_str(cfg->wifi_ssid, sizeof(cfg->wifi_ssid));
  config_terminate_str(cfg->wifi_pass, sizeof(cfg->wifi_pass));
  config_terminate_str(cfg->ap_ssid, sizeof(cfg->ap_ssid));
  config_terminate_str(cfg->ap_pass, sizeof(cfg->ap_pass));

  if (cfg->ap_ssid[0] == '\0')
  {
    config_reset_default_str(cfg->ap_ssid, sizeof(cfg->ap_ssid),
                             defaults.ap_ssid);
  }
  size_t ap_pass_len = strlen(cfg->ap_pass);
  if (ap_pass_len > 0 && ap_pass_len < 8)
  {
    config_reset_default_str(cfg->ap_pass, sizeof(cfg->ap_pass),
                             defaults.ap_pass);
  }
}

static void config_read_str(nvs_handle_t nvs, const char *key, char *out,
                            size_t out_len)
{
  size_t len = out_len;
  esp_err_t err = nvs_get_str(nvs, key, out, &len);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
  {
    ESP_LOGW(TAG, "Ignoring invalid NVS string %s: %s", key,
             esp_err_to_name(err));
  }
}

bool config_load(app_config_t *cfg)
{
  config_set_defaults(cfg);
  config_ensure_mutex();

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGW(TAG, "NVS open failed, using defaults: %s", esp_err_to_name(err));
    config_validate(cfg);
    return false;
  }

  nvs_get_u32(nvs, "gps_int_ms", &cfg->gps_interval_ms);
  nvs_get_u32(nvs, "max_age_s", &cfg->max_gps_age_s);
  nvs_get_u32(nvs, "cfg_win_s", &cfg->config_window_s);
  nvs_get_u32(nvs, "ble_pass", &cfg->ble_passkey);
  int16_t tz = cfg->tz_offset_min;
  int16_t dst = cfg->dst_offset_min;
  uint16_t old_tz = 0;
  uint16_t old_dst = 0;
  if (nvs_get_i16(nvs, "tz_off", &tz) == ESP_OK)
  {
    cfg->tz_offset_min = tz;
  }
  else if (nvs_get_u16(nvs, "tz_off", &old_tz) == ESP_OK)
  {
    cfg->tz_offset_min = (int16_t)old_tz;
  }
  if (nvs_get_i16(nvs, "dst_off", &dst) == ESP_OK)
  {
    cfg->dst_offset_min = dst;
  }
  else if (nvs_get_u16(nvs, "dst_off", &old_dst) == ESP_OK)
  {
    cfg->dst_offset_min = (int16_t)old_dst;
  }

  config_read_str(nvs, "cam_name", cfg->camera_name_prefix,
                  sizeof(cfg->camera_name_prefix));
  config_read_str(nvs, "cam_mac", cfg->camera_mac_prefix,
                  sizeof(cfg->camera_mac_prefix));
  config_read_str(nvs, "wifi_ssid", cfg->wifi_ssid, sizeof(cfg->wifi_ssid));
  config_read_str(nvs, "wifi_pass", cfg->wifi_pass, sizeof(cfg->wifi_pass));
  config_read_str(nvs, "ap_ssid", cfg->ap_ssid, sizeof(cfg->ap_ssid));
  config_read_str(nvs, "ap_pass", cfg->ap_pass, sizeof(cfg->ap_pass));

  nvs_close(nvs);
  config_validate(cfg);
  return true;
}

static bool config_save_unlocked(const app_config_t *cfg)
{
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
    return false;
  }

  err = nvs_set_u32(nvs, "gps_int_ms", cfg->gps_interval_ms);
  if (err == ESP_OK) err = nvs_set_u32(nvs, "max_age_s", cfg->max_gps_age_s);
  if (err == ESP_OK) err = nvs_set_u32(nvs, "cfg_win_s", cfg->config_window_s);
  if (err == ESP_OK) err = nvs_set_u32(nvs, "ble_pass", cfg->ble_passkey);
  if (err == ESP_OK) err = nvs_set_i16(nvs, "tz_off", cfg->tz_offset_min);
  if (err == ESP_OK) err = nvs_set_i16(nvs, "dst_off", cfg->dst_offset_min);
  if (err == ESP_OK) err = nvs_set_str(nvs, "cam_name", cfg->camera_name_prefix);
  if (err == ESP_OK) err = nvs_set_str(nvs, "cam_mac", cfg->camera_mac_prefix);
  if (err == ESP_OK) err = nvs_set_str(nvs, "wifi_ssid", cfg->wifi_ssid);
  if (err == ESP_OK) err = nvs_set_str(nvs, "wifi_pass", cfg->wifi_pass);
  if (err == ESP_OK) err = nvs_set_str(nvs, "ap_ssid", cfg->ap_ssid);
  if (err == ESP_OK) err = nvs_set_str(nvs, "ap_pass", cfg->ap_pass);
  if (err == ESP_OK) {
    err = nvs_commit(nvs);
  }
  nvs_close(nvs);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool config_save(const app_config_t *cfg)
{
  if (!cfg)
  {
    return false;
  }
  app_config_t validated = *cfg;
  config_validate(&validated);
  config_ensure_mutex();
  if (s_config_mutex &&
      xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
  {
    ESP_LOGW(TAG, "Failed to acquire config mutex");
    return false;
  }
  bool ok = config_save_unlocked(&validated);
  if (s_config_mutex)
  {
    xSemaphoreGive(s_config_mutex);
  }
  return ok;
}

bool config_apply(app_config_t *cfg, const app_config_t *new_cfg)
{
  if (!cfg || !new_cfg)
  {
    return false;
  }

  app_config_t validated = *new_cfg;
  config_validate(&validated);
  config_ensure_mutex();
  if (s_config_mutex &&
      xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
  {
    ESP_LOGW(TAG, "Failed to acquire config mutex");
    return false;
  }
  bool ok = config_save_unlocked(&validated);
  if (ok)
  {
    *cfg = validated;
  }
  if (s_config_mutex)
  {
    xSemaphoreGive(s_config_mutex);
  }
  return ok;
}

bool config_get_snapshot(const app_config_t *cfg, app_config_t *out)
{
  if (!cfg || !out)
  {
    return false;
  }

  config_ensure_mutex();
  if (s_config_mutex &&
      xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
  {
    return false;
  }
  *out = *cfg;
  if (s_config_mutex)
  {
    xSemaphoreGive(s_config_mutex);
  }
  return true;
}
