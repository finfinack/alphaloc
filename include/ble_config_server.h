#ifndef ALPHALOC_BLE_CONFIG_SERVER_H
#define ALPHALOC_BLE_CONFIG_SERVER_H

#include "config.h"

void ble_config_server_register(app_config_t *cfg);
void ble_config_server_on_sync(void);
void ble_config_server_start(void);
void ble_config_server_stop(void);

#endif
