#include "gps.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define GPS_UART_BUF_SIZE 2048
#define GPS_LINE_MAX 128

static const char *TAG = "gps";

#ifndef ALPHALOC_VERBOSE
#define ALPHALOC_VERBOSE 0
#endif

#if ALPHALOC_VERBOSE
#define VLOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define VLOGI(...) ((void)0)
#endif

static gps_fix_t s_latest_fix;
static SemaphoreHandle_t s_fix_mutex;
static gps_config_t s_cfg;

static double parse_deg_min(const char *value) {
  if (value == NULL || value[0] == '\0') {
    return 0.0;
  }
  double v = strtod(value, NULL);
  double deg = floor(v / 100.0);
  double min = v - (deg * 100.0);
  return deg + (min / 60.0);
}

static bool parse_rmc(const char *line, gps_fix_t *out) {
  char buf[GPS_LINE_MAX];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  const char *fields[20] = {0};
  int field_count = 0;
  char *saveptr = NULL;
  char *token = strtok_r(buf, ",", &saveptr);
  while (token && field_count < 20) {
    fields[field_count++] = token;
    token = strtok_r(NULL, ",", &saveptr);
  }

  if (field_count < 10) {
    return false;
  }

  if (fields[2][0] != 'A') {
    return false;
  }

  out->time_valid = false;
  out->year = 0;
  out->month = 0;
  out->day = 0;
  out->hour = 0;
  out->minute = 0;
  out->second = 0;
  bool time_ok = false;
  bool date_ok = false;
  if (fields[1][0] != '\0') {
    int hh = 0, mm = 0, ss = 0;
    if (sscanf(fields[1], "%2d%2d%2d", &hh, &mm, &ss) == 3) {
      out->hour = (uint8_t)hh;
      out->minute = (uint8_t)mm;
      out->second = (uint8_t)ss;
      time_ok = true;
    }
  }

  if (fields[9][0] != '\0') {
    int dd = 0, mm = 0, yy = 0;
    if (sscanf(fields[9], "%2d%2d%2d", &dd, &mm, &yy) == 3) {
      out->day = (uint8_t)dd;
      out->month = (uint8_t)mm;
      out->year = (uint16_t)(2000 + yy);
      date_ok = true;
    }
  }
  out->time_valid = time_ok;

  double lat = parse_deg_min(fields[3]);
  if (fields[4][0] == 'S') {
    lat = -lat;
  }

  double lon = parse_deg_min(fields[5]);
  if (fields[6][0] == 'W') {
    lon = -lon;
  }

  out->lat_deg = lat;
  out->lon_deg = lon;
  out->valid = true;
  out->last_fix_time_us = esp_timer_get_time();
  return true;
}

static void update_fix(const gps_fix_t *fix, bool has_fix) {
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_latest_fix.last_update_time_us = esp_timer_get_time();
    if (has_fix) {
      const bool date_present = (fix->year != 0 && fix->month != 0 && fix->day != 0);
      s_latest_fix.lat_deg = fix->lat_deg;
      s_latest_fix.lon_deg = fix->lon_deg;
      s_latest_fix.altitude_m = fix->altitude_m;
      s_latest_fix.valid = true;
      s_latest_fix.last_fix_time_us = fix->last_fix_time_us;
      if (fix->time_valid) {
        s_latest_fix.hour = fix->hour;
        s_latest_fix.minute = fix->minute;
        s_latest_fix.second = fix->second;
        s_latest_fix.time_valid = true;
      }
      if (date_present) {
        s_latest_fix.year = fix->year;
        s_latest_fix.month = fix->month;
        s_latest_fix.day = fix->day;
      }
      VLOGI("Fix lat=%.7f lon=%.7f time=%04u-%02u-%02u %02u:%02u:%02u",
            fix->lat_deg, fix->lon_deg,
            fix->year, fix->month, fix->day,
            fix->hour, fix->minute, fix->second);
    } else {
      VLOGI("No valid fix");
    }
    xSemaphoreGive(s_fix_mutex);
  }
}

static void update_time_date(const gps_fix_t *fix, bool time_present, bool date_present) {
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_latest_fix.last_update_time_us = esp_timer_get_time();
    if (time_present) {
      s_latest_fix.hour = fix->hour;
      s_latest_fix.minute = fix->minute;
      s_latest_fix.second = fix->second;
      s_latest_fix.time_valid = true;
    }
    if (date_present) {
      s_latest_fix.year = fix->year;
      s_latest_fix.month = fix->month;
      s_latest_fix.day = fix->day;
    }
    xSemaphoreGive(s_fix_mutex);
  }
}

static bool parse_zda(const char *line, gps_fix_t *out, bool *time_ok, bool *date_ok) {
  char buf[GPS_LINE_MAX];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  const char *fields[8] = {0};
  int field_count = 0;
  char *saveptr = NULL;
  char *token = strtok_r(buf, ",", &saveptr);
  while (token && field_count < 8) {
    fields[field_count++] = token;
    token = strtok_r(NULL, ",", &saveptr);
  }

  if (field_count < 5) {
    return false;
  }

  *time_ok = false;
  *date_ok = false;
  if (fields[1][0] != '\0') {
    int hh = 0, mm = 0, ss = 0;
    if (sscanf(fields[1], "%2d%2d%2d", &hh, &mm, &ss) == 3) {
      out->hour = (uint8_t)hh;
      out->minute = (uint8_t)mm;
      out->second = (uint8_t)ss;
      *time_ok = true;
    }
  }

  if (fields[2][0] != '\0' && fields[3][0] != '\0' && fields[4][0] != '\0') {
    int dd = 0, mm = 0, yyyy = 0;
    if (sscanf(fields[2], "%2d", &dd) == 1 &&
        sscanf(fields[3], "%2d", &mm) == 1 &&
        sscanf(fields[4], "%4d", &yyyy) == 1) {
      out->day = (uint8_t)dd;
      out->month = (uint8_t)mm;
      out->year = (uint16_t)yyyy;
      *date_ok = true;
    }
  }

  return *time_ok || *date_ok;
}

static void gps_task(void *arg) {
  uint8_t rx_buf[GPS_UART_BUF_SIZE];
  char line_buf[GPS_LINE_MAX];
  size_t line_len = 0;

  while (true) {
    int len = uart_read_bytes(s_cfg.uart_num, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(200));
    if (len <= 0) {
      vTaskDelay(pdMS_TO_TICKS(s_cfg.update_interval_ms));
      continue;
    }

    for (int i = 0; i < len; ++i) {
      char c = (char)rx_buf[i];
      if (c == '\n' || c == '\r') {
        if (line_len == 0) {
          continue;
        }
        line_buf[line_len] = '\0';
        ESP_LOGI(TAG, "NMEA: %s", line_buf);
        gps_fix_t fix = {0};
        if (strncmp(line_buf, "$GPRMC", 6) == 0 || strncmp(line_buf, "$GNRMC", 6) == 0) {
          if (parse_rmc(line_buf, &fix)) {
            update_fix(&fix, true);
          } else {
            update_fix(&fix, false);
          }
        } else if (strncmp(line_buf, "$GPZDA", 6) == 0 || strncmp(line_buf, "$GNZDA", 6) == 0) {
          bool time_ok = false;
          bool date_ok = false;
          if (parse_zda(line_buf, &fix, &time_ok, &date_ok)) {
            update_time_date(&fix, time_ok, date_ok);
          }
        }
        line_len = 0;
        continue;
      }

      if (isprint((unsigned char)c) && line_len < sizeof(line_buf) - 1) {
        line_buf[line_len++] = c;
      }
    }
  }
}

void gps_init(const gps_config_t *cfg) {
  s_cfg = *cfg;
  s_fix_mutex = xSemaphoreCreateMutex();
  memset(&s_latest_fix, 0, sizeof(s_latest_fix));

  uart_config_t uart_cfg = {
      .baud_rate = s_cfg.baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(uart_driver_install(s_cfg.uart_num, GPS_UART_BUF_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num, s_cfg.tx_pin, s_cfg.rx_pin, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE));

  xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
  ESP_LOGI(TAG, "GPS task started");
}

bool gps_get_latest(gps_fix_t *out_fix) {
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  *out_fix = s_latest_fix;
  xSemaphoreGive(s_fix_mutex);
  return true;
}
