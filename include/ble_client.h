#ifndef ALPHALOC_BLE_CLIENT_H
#define ALPHALOC_BLE_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "gps.h"
#include "config.h"

struct ble_gap_event;

typedef void (*ble_focus_cb_t)(void *ctx);

void ble_client_init(const app_config_t *cfg);
void ble_client_set_focus_callback(ble_focus_cb_t cb, void *ctx);
bool ble_client_is_connected(void);
bool ble_client_send_location(const gps_fix_t *fix);
int ble_client_gap_event_cb(struct ble_gap_event *event, void *arg);

#endif
