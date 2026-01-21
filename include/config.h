#ifndef ALPHALOC_CONFIG_H
#define ALPHALOC_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_STR_MAX_32 32
#define CONFIG_STR_MAX_64 64
#define CONFIG_STR_MAX_18 18

typedef enum {
  APP_WIFI_MODE_AP = 0,
  APP_WIFI_MODE_STA = 1,
} app_wifi_mode_t;

typedef struct {
  uint32_t gps_interval_ms;
  uint32_t max_gps_age_s;
  uint32_t config_window_s;
  char camera_name_prefix[CONFIG_STR_MAX_32];
  char camera_mac_prefix[CONFIG_STR_MAX_18];
  uint32_t ble_passkey;
  uint16_t tz_offset_min;
  uint16_t dst_offset_min;
  app_wifi_mode_t wifi_mode;
  char wifi_ssid[CONFIG_STR_MAX_32];
  char wifi_pass[CONFIG_STR_MAX_64];
  char ap_ssid[CONFIG_STR_MAX_32];
  char ap_pass[CONFIG_STR_MAX_64];
} app_config_t;

void config_set_defaults(app_config_t *cfg);
bool config_load(app_config_t *cfg);
bool config_save(const app_config_t *cfg);

#endif
