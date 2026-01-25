#include <string.h>

#include "ble_client.h"
#include "ble_config_server.h"
#include "config.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef ALPHALOC_NEOPIXEL_PIN
#include "neopixel.h"
#endif

#ifndef ALPHALOC_WIFI_WEB
#define ALPHALOC_WIFI_WEB 1
#endif

#if ALPHALOC_WIFI_WEB
#include "wifi_web.h"
#endif

#define GPS_UART_NUM UART_NUM_1
#ifndef GPS_UART_TX_PIN
#error "GPS_UART_TX_PIN must be set via build_flags"
#endif
#ifndef GPS_UART_RX_PIN
#error "GPS_UART_RX_PIN must be set via build_flags"
#endif
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
#if ALPHALOC_WIFI_WEB
  wifi_web_start(cfg);
#endif
  vTaskDelay(pdMS_TO_TICKS(cfg->config_window_s * 1000));
  ble_config_server_stop();
#if ALPHALOC_WIFI_WEB
  wifi_web_stop();
#endif
  vTaskDelete(NULL);
}

#ifdef ALPHALOC_NEOPIXEL_PIN
static void status_led_task(void *arg)
{
  const app_config_t *cfg = (const app_config_t *)arg;
  const uint8_t brightness = 8;
  const TickType_t on_ms = pdMS_TO_TICKS(150);
  const TickType_t off_ms = pdMS_TO_TICKS(150);
  const TickType_t cycle_ms = pdMS_TO_TICKS(3000);

  neopixel_init(ALPHALOC_NEOPIXEL_PIN, brightness);

  while (true)
  {
    TickType_t cycle_start = xTaskGetTickCount();

    // Camera status: green if connected, red if not.
    bool camera_connected = ble_client_is_connected();
    if (camera_connected)
    {
      neopixel_set_rgb(0, 255, 0);
    }
    else
    {
      neopixel_set_rgb(255, 0, 0);
    }
    vTaskDelay(on_ms);
    neopixel_set_rgb(0, 0, 0);
    vTaskDelay(off_ms);

    // GPS status: green if fix is valid, red if not.
    gps_fix_t fix;
    bool gps_ok = gps_get_latest(&fix) && fix.valid;
    if (gps_ok)
    {
      neopixel_set_rgb(0, 255, 0);
    }
    else
    {
      neopixel_set_rgb(255, 0, 0);
    }
    vTaskDelay(on_ms);
    neopixel_set_rgb(0, 0, 0);
    vTaskDelay(off_ms);

    // Wi-Fi status: blue if web server/AP active.
#if ALPHALOC_WIFI_WEB
    if (cfg->config_window_s > 0)
    {
      neopixel_set_rgb(0, 0, 255);
      vTaskDelay(on_ms);
      neopixel_set_rgb(0, 0, 0);
      vTaskDelay(off_ms);
    }
#endif

    TickType_t elapsed = xTaskGetTickCount() - cycle_start;
    if (elapsed < cycle_ms)
    {
      vTaskDelay(cycle_ms - elapsed);
    }
  }
}
#endif

void app_main(void)
{
  ESP_LOGI(TAG, "AlphaLoc starting");
#ifdef ALPHALOC_STEMMA_QT_DISABLE_PIN
  // Disable STEMMA QT power for lower power usage when unused.
  gpio_config_t stemma_cfg = {
      .pin_bit_mask = 1ULL << ALPHALOC_STEMMA_QT_DISABLE_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&stemma_cfg));
  ESP_ERROR_CHECK(gpio_set_level(ALPHALOC_STEMMA_QT_DISABLE_PIN, 0));
#endif
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
#ifdef ALPHALOC_FACTORY_RESET
    if (ALPHALOC_FACTORY_RESET)
    {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
      ESP_LOGW(TAG, "NVS needs erase but factory reset disabled; keeping data");
    }
#else
    ESP_LOGW(TAG, "NVS needs erase but factory reset flag not set; keeping data");
#endif
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
#ifdef ALPHALOC_NEOPIXEL_PIN
  xTaskCreate(status_led_task, "status_led", 3072, &s_cfg, 1, NULL);
#endif
  ESP_LOGI(TAG, "AlphaLoc started");
}
