#include "config.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE "alphaloc"

static const char *TAG = "config";
static SemaphoreHandle_t s_config_mutex = NULL;

void config_set_defaults(app_config_t *cfg)
{
  memset(cfg, 0, sizeof(*cfg));
  cfg->gps_interval_ms = 5000;
  cfg->max_gps_age_s = 300;
  cfg->config_window_s = 300;
  cfg->ble_passkey = 123456;
  cfg->tz_offset_min = 60;
  cfg->dst_offset_min = 60;
  // cfg->wifi_mode = APP_WIFI_MODE_STA;
  cfg->wifi_mode = APP_WIFI_MODE_AP;
  strncpy(cfg->camera_name_prefix, "SonyA7",
          sizeof(cfg->camera_name_prefix) - 1);
  cfg->camera_mac_prefix[0] = '\0';
  strncpy(cfg->ap_ssid, "AlphaLoc", sizeof(cfg->ap_ssid) - 1);
  strncpy(cfg->ap_pass, "alphaloc1234", sizeof(cfg->ap_pass) - 1);
  strncpy(cfg->wifi_ssid, "WiFi", sizeof(cfg->wifi_ssid) - 1);
  strncpy(cfg->wifi_pass, "changeme", sizeof(cfg->wifi_pass) - 1);
}

static void config_read_str(nvs_handle_t nvs, const char *key, char *out,
                            size_t out_len)
{
  size_t len = out_len;
  esp_err_t err = nvs_get_str(nvs, key, out, &len);
  if (err != ESP_OK)
  {
    out[0] = '\0';
  }
}

bool config_load(app_config_t *cfg)
{
  config_set_defaults(cfg);

  // Initialize mutex on first load
  if (s_config_mutex == NULL)
  {
    s_config_mutex = xSemaphoreCreateMutex();
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGW(TAG, "NVS open failed, using defaults: %s", esp_err_to_name(err));
    return false;
  }

  nvs_get_u32(nvs, "gps_int_ms", &cfg->gps_interval_ms);
  nvs_get_u32(nvs, "max_age_s", &cfg->max_gps_age_s);
  nvs_get_u32(nvs, "cfg_win_s", &cfg->config_window_s);
  nvs_get_u32(nvs, "ble_pass", &cfg->ble_passkey);
  uint16_t tz = cfg->tz_offset_min;
  uint16_t dst = cfg->dst_offset_min;
  if (nvs_get_u16(nvs, "tz_off", &tz) == ESP_OK)
  {
    cfg->tz_offset_min = tz;
  }
  if (nvs_get_u16(nvs, "dst_off", &dst) == ESP_OK)
  {
    cfg->dst_offset_min = dst;
  }

  uint8_t wifi_mode = cfg->wifi_mode;
  if (nvs_get_u8(nvs, "wifi_mode", &wifi_mode) == ESP_OK)
  {
    cfg->wifi_mode = (app_wifi_mode_t)wifi_mode;
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
  return true;
}

bool config_save(const app_config_t *cfg)
{
  // Protect concurrent config save operations
  if (s_config_mutex &&
      xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
  {
    ESP_LOGW(TAG, "Failed to acquire config mutex");
    return false;
  }

  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
    if (s_config_mutex)
    {
      xSemaphoreGive(s_config_mutex);
    }
    return false;
  }

  nvs_set_u32(nvs, "gps_int_ms", cfg->gps_interval_ms);
  nvs_set_u32(nvs, "max_age_s", cfg->max_gps_age_s);
  nvs_set_u32(nvs, "cfg_win_s", cfg->config_window_s);
  nvs_set_u32(nvs, "ble_pass", cfg->ble_passkey);
  nvs_set_u16(nvs, "tz_off", cfg->tz_offset_min);
  nvs_set_u16(nvs, "dst_off", cfg->dst_offset_min);
  nvs_set_u8(nvs, "wifi_mode", (uint8_t)cfg->wifi_mode);

  nvs_set_str(nvs, "cam_name", cfg->camera_name_prefix);
  nvs_set_str(nvs, "cam_mac", cfg->camera_mac_prefix);
  nvs_set_str(nvs, "wifi_ssid", cfg->wifi_ssid);
  nvs_set_str(nvs, "wifi_pass", cfg->wifi_pass);
  nvs_set_str(nvs, "ap_ssid", cfg->ap_ssid);
  nvs_set_str(nvs, "ap_pass", cfg->ap_pass);

  err = nvs_commit(nvs);
  nvs_close(nvs);

  if (s_config_mutex)
  {
    xSemaphoreGive(s_config_mutex);
  }

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}
