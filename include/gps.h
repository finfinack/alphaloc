#ifndef ALPHALOC_GPS_H
#define ALPHALOC_GPS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  double lat_deg;
  double lon_deg;
  double altitude_m;
  bool valid;
  bool time_valid;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  int64_t last_fix_time_us;
  int64_t last_update_time_us;
} gps_fix_t;

typedef enum {
  GPS_CONSTELLATION_NONE = 0,
  GPS_CONSTELLATION_GPS = 1 << 0,
  GPS_CONSTELLATION_GLONASS = 1 << 1,
} gps_constellation_t;

typedef struct {
  bool has_lock;
  uint8_t satellites;
  gps_constellation_t constellations;
} gps_status_t;

typedef struct {
  int uart_num;
  int tx_pin;
  int rx_pin;
  int baud_rate;
  uint32_t update_interval_ms;
} gps_config_t;

void gps_init(const gps_config_t *cfg);
bool gps_get_latest(gps_fix_t *out_fix);
bool gps_get_status(gps_status_t *out_status);

#endif
