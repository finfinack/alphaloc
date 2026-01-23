#include <string.h>

#include "ble_client.h"
#include "ble_config_server.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "wifi_web.h"

#define GPS_UART_NUM UART_NUM_1
// ESP32 (Feather V2)
#define GPS_UART_TX_PIN 8
#define GPS_UART_RX_PIN 7
// ESP32-C6 (DFRobot)
// #define GPS_UART_TX_PIN 5
// #define GPS_UART_RX_PIN 4
#define GPS_UART_BAUD 9600

#ifndef ALPHALOC_FAKE_GPS
#define ALPHALOC_FAKE_GPS 0
#endif

#define FAKE_LAT_DEG 48.137154
#define FAKE_LON_DEG 11.576124
#define FAKE_YEAR 2024
#define FAKE_MONTH 1
#define FAKE_DAY 1
#define FAKE_HOUR 12
#define FAKE_MINUTE 0
#define FAKE_SECOND 0

static const char *TAG = "main";
static app_config_t s_cfg;

static bool get_location_for_send(gps_fix_t *out_fix)
{
  gps_fix_t fix;
  if (gps_get_latest(&fix) && fix.valid)
  {
    *out_fix = fix;
    return true;
  }
#if ALPHALOC_FAKE_GPS
  memset(&fix, 0, sizeof(fix));
  fix.lat_deg = FAKE_LAT_DEG;
  fix.lon_deg = FAKE_LON_DEG;
  fix.valid = true;
  fix.time_valid = true;
  fix.year = FAKE_YEAR;
  fix.month = FAKE_MONTH;
  fix.day = FAKE_DAY;
  fix.hour = FAKE_HOUR;
  fix.minute = FAKE_MINUTE;
  fix.second = FAKE_SECOND;
  fix.last_fix_time_us = esp_timer_get_time();
  fix.last_update_time_us = fix.last_fix_time_us;
  *out_fix = fix;
  return true;
#else
  return false;
#endif
}

static void focus_update_cb(void *ctx)
{
  app_config_t *cfg = (app_config_t *)ctx;
  gps_fix_t fix;
  if (!get_location_for_send(&fix))
  {
    return;
  }
  int64_t now = esp_timer_get_time();
  if ((now - fix.last_fix_time_us) > (int64_t)cfg->max_gps_age_s * 1000000LL)
  {
    return;
  }
  ble_client_send_location(&fix);
}

static void location_publisher_task(void *arg)
{
  const app_config_t *cfg = (const app_config_t *)arg;
  while (true)
  {
    gps_fix_t fix;
    if (get_location_for_send(&fix))
    {
      int64_t now = esp_timer_get_time();
      if ((now - fix.last_fix_time_us) <= (int64_t)cfg->max_gps_age_s * 1000000LL)
      {
        ble_client_send_location(&fix);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(cfg->gps_interval_ms));
  }
}

static void config_window_task(void *arg)
{
  app_config_t *cfg = (app_config_t *)arg;
  ble_config_server_start();
  wifi_web_start(cfg);
  vTaskDelay(pdMS_TO_TICKS(cfg->config_window_s * 1000));
  ble_config_server_stop();
  wifi_web_stop();
  vTaskDelete(NULL);
}

void app_main(void)
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  config_load(&s_cfg);

  gps_config_t gps_cfg = {
      .uart_num = GPS_UART_NUM,
      .tx_pin = GPS_UART_TX_PIN,
      .rx_pin = GPS_UART_RX_PIN,
      .baud_rate = GPS_UART_BAUD,
      .update_interval_ms = s_cfg.gps_interval_ms,
  };
  gps_init(&gps_cfg);

  ble_client_init(&s_cfg);
  ble_client_set_focus_callback(focus_update_cb, &s_cfg);

  xTaskCreate(location_publisher_task, "location_pub", 4096, &s_cfg, 5, NULL);
  if (s_cfg.config_window_s > 0)
  {
    xTaskCreate(config_window_task, "config_window", 4096, &s_cfg, 5, NULL);
  }
  ESP_LOGI(TAG, "AlphaLoc started");
}
