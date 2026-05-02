#include "gps.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define GPS_UART_BUF_SIZE 2048
#define GPS_LINE_MAX 128

static const char *TAG = "gps";

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

#ifndef ALPHALOC_VERBOSE
#define ALPHALOC_VERBOSE 0
#endif

#ifndef ALPHALOC_LOG_NMEA
#define ALPHALOC_LOG_NMEA 0
#endif

#if ALPHALOC_VERBOSE
#define VLOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define VLOGI(...) ((void)0)
#endif

#if ALPHALOC_LOG_NMEA
#define NMEALOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define NMEALOGI(...) ((void)0)
#endif

static gps_fix_t s_latest_fix;
static SemaphoreHandle_t s_fix_mutex;
static gps_config_t s_cfg;
static gps_status_t s_status;
static int64_t s_last_no_fix_log_us;

static int hex_value(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  return -1;
}

static bool nmea_checksum_valid(const char *line) {
  if (!line || line[0] != '$') {
    return false;
  }
  const char *star = strchr(line, '*');
  if (!star || !star[1] || !star[2] || star[3] != '\0') {
    return false;
  }
  int high = hex_value(star[1]);
  int low = hex_value(star[2]);
  if (high < 0 || low < 0) {
    return false;
  }
  uint8_t checksum = 0;
  for (const char *p = line + 1; p < star; ++p) {
    checksum ^= (uint8_t)*p;
  }
  return checksum == (uint8_t)((high << 4) | low);
}

static bool parse_int_range(const char *value, int min, int max, int *out) {
  if (!value || value[0] == '\0') {
    return false;
  }
  char *end = NULL;
  long parsed = strtol(value, &end, 10);
  if (end == value || *end != '\0' || parsed < min || parsed > max) {
    return false;
  }
  *out = (int)parsed;
  return true;
}

static bool parse_time_hms(const char *value, uint8_t *hour, uint8_t *minute,
                           uint8_t *second) {
  if (!value || strlen(value) < 6) {
    return false;
  }
  for (int i = 0; i < 6; ++i) {
    if (!isdigit((unsigned char)value[i])) {
      return false;
    }
  }
  if (value[6] != '\0' && value[6] != '.') {
    return false;
  }
  int hh = (value[0] - '0') * 10 + (value[1] - '0');
  int mm = (value[2] - '0') * 10 + (value[3] - '0');
  int ss = (value[4] - '0') * 10 + (value[5] - '0');
  if (hh > 23 || mm > 59 || ss > 60) {
    return false;
  }
  *hour = (uint8_t)hh;
  *minute = (uint8_t)mm;
  *second = (uint8_t)ss;
  return true;
}

static bool date_valid(int year, int month, int day) {
  static const uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30,
                                          31, 31, 30, 31, 30, 31};
  if (year < 2000 || year > 2099 || month < 1 || month > 12 || day < 1) {
    return false;
  }
  int max_day = days_in_month[month - 1];
  bool leap = ((year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0));
  if (month == 2 && leap) {
    max_day = 29;
  }
  return day <= max_day;
}

static int split_csv(char *buf, const char *fields[], int max_fields) {
  int count = 0;
  char *p = buf;
  while (count < max_fields) {
    fields[count++] = p;
    char *comma = strchr(p, ',');
    if (!comma) {
      break;
    }
    *comma = '\0';
    p = comma + 1;
  }
  return count;
}

static void update_status_gga(const char *line) {
  // GGA fields: 6=fix quality, 7=satellites, 8=HDOP
  char buf[GPS_LINE_MAX];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  const char *fields[12] = {0};
  int field_count = split_csv(buf, fields, 12);
  if (field_count < 9) {
    return;
  }

  int fix_quality = 0;
  int sats = 0;
  if (!parse_int_range(fields[6], 0, 8, &fix_quality) ||
      !parse_int_range(fields[7], 0, 64, &sats)) {
    return;
  }
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_status.has_lock = (fix_quality > 0);
    s_status.satellites = (uint8_t)sats;
    if (strncmp(line, "$GPGGA", 6) == 0) {
      s_status.constellations = GPS_CONSTELLATION_GPS;
    } else if (strncmp(line, "$GNGGA", 6) == 0) {
      s_status.constellations =
          (GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GLONASS);
    }
    xSemaphoreGive(s_fix_mutex);
  }
}

static bool parse_deg_min(const char *value, const char *hemisphere,
                          bool is_latitude, double *out) {
  if (value == NULL || value[0] == '\0') {
    return false;
  }
  char *end = NULL;
  double v = strtod(value, &end);
  if (end == value || *end != '\0') {
    return false;
  }
  double deg = floor(v / 100.0);
  double min = v - (deg * 100.0);
  if (min < 0.0 || min >= 60.0) {
    return false;
  }
  double max_deg = is_latitude ? 90.0 : 180.0;
  if (deg < 0.0 || deg > max_deg) {
    return false;
  }
  double parsed = deg + (min / 60.0);
  if (parsed > max_deg) {
    return false;
  }
  if (hemisphere == NULL || hemisphere[0] == '\0' || hemisphere[1] != '\0') {
    return false;
  }
  if (is_latitude) {
    if (hemisphere[0] == 'S') {
      parsed = -parsed;
    } else if (hemisphere[0] != 'N') {
      return false;
    }
  } else {
    if (hemisphere[0] == 'W') {
      parsed = -parsed;
    } else if (hemisphere[0] != 'E') {
      return false;
    }
  }
  *out = parsed;
  return true;
}

static bool parse_rmc(const char *line, gps_fix_t *out) {
  char buf[GPS_LINE_MAX];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  const char *fields[20] = {0};
  int field_count = split_csv(buf, fields, 20);

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
  if (fields[1][0] != '\0') {
    if (parse_time_hms(fields[1], &out->hour, &out->minute, &out->second)) {
      time_ok = true;
    }
  }

  if (fields[9][0] != '\0') {
    int dd = 0, mm = 0, yy = 0;
    char extra = '\0';
    if (sscanf(fields[9], "%2d%2d%2d%c", &dd, &mm, &yy, &extra) == 3 &&
        date_valid(2000 + yy, mm, dd)) {
      out->day = (uint8_t)dd;
      out->month = (uint8_t)mm;
      out->year = (uint16_t)(2000 + yy);
    }
  }
  out->time_valid = time_ok;

  double lat = 0.0;
  double lon = 0.0;
  if (!parse_deg_min(fields[3], fields[4], true, &lat) ||
      !parse_deg_min(fields[5], fields[6], false, &lon)) {
    return false;
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
      const bool date_present =
          (fix->year != 0 && fix->month != 0 && fix->day != 0);
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
            fix->lat_deg, fix->lon_deg, (unsigned)fix->year,
            (unsigned)fix->month, (unsigned)fix->day, (unsigned)fix->hour,
            (unsigned)fix->minute, (unsigned)fix->second);
    } else {
      s_latest_fix.valid = false;
      int64_t now = esp_timer_get_time();
      if (now - s_last_no_fix_log_us > 5000000) {
        s_last_no_fix_log_us = now;
        VLOGI("No valid fix");
      }
    }
    xSemaphoreGive(s_fix_mutex);
  }
}

static void update_time_date(const gps_fix_t *fix, bool time_present,
                             bool date_present) {
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

static bool parse_zda(const char *line, gps_fix_t *out, bool *time_ok,
                      bool *date_ok) {
  char buf[GPS_LINE_MAX];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  const char *fields[8] = {0};
  int field_count = split_csv(buf, fields, 8);

  if (field_count < 5) {
    return false;
  }

  *time_ok = false;
  *date_ok = false;
  if (fields[1][0] != '\0') {
    if (parse_time_hms(fields[1], &out->hour, &out->minute, &out->second)) {
      *time_ok = true;
    }
  }

  if (fields[2][0] != '\0' && fields[3][0] != '\0' && fields[4][0] != '\0') {
    int dd = 0, mm = 0, yyyy = 0;
    if (parse_int_range(fields[2], 1, 31, &dd) &&
        parse_int_range(fields[3], 1, 12, &mm) &&
        parse_int_range(fields[4], 2000, 2099, &yyyy) &&
        date_valid(yyyy, mm, dd)) {
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
    int len = uart_read_bytes(s_cfg.uart_num, rx_buf, sizeof(rx_buf),
                              pdMS_TO_TICKS(200));

    // Add error handling for UART read failures
    if (len < 0) {
      ESP_LOGE(TAG, "UART read error: %d", len);
      vTaskDelay(pdMS_TO_TICKS(100));
      line_len = 0; // Reset buffer on error
      continue;
    }

    if (len == 0) {
      vTaskDelay(pdMS_TO_TICKS(s_cfg.update_interval_ms));
      continue;
    }

#if ALPHALOC_LOG_NMEA
    ESP_LOG_BUFFER_CHAR(TAG, rx_buf, len);
#endif

    for (int i = 0; i < len; ++i) {
      char c = (char)rx_buf[i];
      if (c == '\n' || c == '\r') {
        if (line_len == 0) {
          continue;
        }
        line_buf[line_len] = '\0';

        // Validate minimum NMEA sentence length before parsing
        if (line_len >= 10 && nmea_checksum_valid(line_buf)) {
          NMEALOGI("NMEA: %s", line_buf);
          if (strncmp(line_buf, "$GPGGA", 6) == 0 ||
              strncmp(line_buf, "$GNGGA", 6) == 0) {
            update_status_gga(line_buf);
          }
          gps_fix_t fix = {0};
          if (strncmp(line_buf, "$GPRMC", 6) == 0 ||
              strncmp(line_buf, "$GNRMC", 6) == 0) {
            if (parse_rmc(line_buf, &fix)) {
              update_fix(&fix, true);
            } else {
              update_fix(&fix, false);
            }
          } else if (strncmp(line_buf, "$GPZDA", 6) == 0 ||
                     strncmp(line_buf, "$GNZDA", 6) == 0) {
            bool time_ok = false;
            bool date_ok = false;
            if (parse_zda(line_buf, &fix, &time_ok, &date_ok)) {
              update_time_date(&fix, time_ok, date_ok);
            }
          }
        }
        line_len = 0;
        continue;
      }

      // Add bounds check to prevent buffer overflow
      if (isprint((unsigned char)c) && line_len < sizeof(line_buf) - 1) {
        line_buf[line_len++] = c;
      } else if (line_len >= sizeof(line_buf) - 1) {
        // Buffer full, discard this sentence
        line_len = 0;
      }
    }
  }
}

void gps_init(const gps_config_t *cfg) {
  if (!cfg) {
    ESP_LOGE(TAG, "GPS config is null");
    return;
  }
  s_cfg = *cfg;
  s_fix_mutex = xSemaphoreCreateMutex();
  if (s_fix_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create GPS mutex");
    return;
  }
  memset(&s_latest_fix, 0, sizeof(s_latest_fix));
  memset(&s_status, 0, sizeof(s_status));
  s_last_no_fix_log_us = 0;

#if ALPHALOC_FAKE_GPS
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    const int64_t now = esp_timer_get_time();
    s_latest_fix.lat_deg = FAKE_LAT_DEG;
    s_latest_fix.lon_deg = FAKE_LON_DEG;
    s_latest_fix.valid = true;
    s_latest_fix.time_valid = true;
    s_latest_fix.year = FAKE_YEAR;
    s_latest_fix.month = FAKE_MONTH;
    s_latest_fix.day = FAKE_DAY;
    s_latest_fix.hour = FAKE_HOUR;
    s_latest_fix.minute = FAKE_MINUTE;
    s_latest_fix.second = FAKE_SECOND;
    s_latest_fix.last_fix_time_us = now;
    s_latest_fix.last_update_time_us = now;
    s_status.has_lock = true;
    s_status.satellites = 8;
    s_status.constellations = GPS_CONSTELLATION_GPS;
    xSemaphoreGive(s_fix_mutex);
  }
  ESP_LOGI(TAG, "Fake GPS enabled");
  return;
#endif

  uart_config_t uart_cfg = {
      .baud_rate = s_cfg.baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  ESP_ERROR_CHECK(
      uart_driver_install(s_cfg.uart_num, GPS_UART_BUF_SIZE, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(s_cfg.uart_num, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(s_cfg.uart_num, s_cfg.tx_pin, s_cfg.rx_pin,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Bump stack to avoid overflow when parsing/logging NMEA sentences.
  BaseType_t ret = xTaskCreate(gps_task, "gps_task", 6144, NULL, 5, NULL);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create GPS task");
    return;
  }
  ESP_LOGI(TAG, "GPS task started");
}

bool gps_get_latest(gps_fix_t *out_fix) {
  if (!out_fix || !s_fix_mutex) {
    return false;
  }
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  *out_fix = s_latest_fix;
  xSemaphoreGive(s_fix_mutex);
  return true;
}

bool gps_get_status(gps_status_t *out_status) {
  if (!out_status || !s_fix_mutex) {
    return false;
  }
  if (xSemaphoreTake(s_fix_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }
  *out_status = s_status;
  xSemaphoreGive(s_fix_mutex);
  return true;
}
