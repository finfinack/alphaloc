#ifndef ALPHALOC_BATTERY_H
#define ALPHALOC_BATTERY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BATTERY_MONITOR_NONE = 0,
  BATTERY_MONITOR_MAX17048,
  BATTERY_MONITOR_LC709203F,
} battery_monitor_t;

typedef struct {
  bool valid;
  float voltage_v;
  float percent;
  battery_monitor_t monitor;
  int64_t last_update_us;
} battery_status_t;

bool battery_init(void);
bool battery_read_now(void);
bool battery_get_status(battery_status_t *out);

#endif
