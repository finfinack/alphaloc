#include "battery.h"

#include <string.h>

#ifndef ALPHALOC_BATTERY_MONITOR
#define ALPHALOC_BATTERY_MONITOR 0
#endif
#ifndef ALPHALOC_VERBOSE
#define ALPHALOC_VERBOSE 0
#endif

#if ALPHALOC_BATTERY_MONITOR

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

static battery_status_t s_status;
static bool s_inited = false;
static i2c_port_t s_port = I2C_NUM_0;
#ifdef ALPHALOC_BATTERY_I2C_POWER_PIN
static const int s_i2c_power_pin = ALPHALOC_BATTERY_I2C_POWER_PIN;
#endif

#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 100

#define MAX17048_ADDR 0x36
#define LC709203F_ADDR 0x0B

static bool i2c_read_reg16(uint8_t addr, uint8_t reg, uint16_t *out) {
  uint8_t data[2] = {0};
  esp_err_t err = i2c_master_write_read_device(
      s_port, addr, &reg, 1, data, sizeof(data),
      pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  if (err != ESP_OK) {
    return false;
  }
  *out = ((uint16_t)data[0] << 8) | data[1];
  return true;
}

static bool read_max17048(float *out_voltage, float *out_percent) {
  uint16_t vcell = 0;
  uint16_t soc = 0;
  if (!i2c_read_reg16(MAX17048_ADDR, 0x02, &vcell)) {
    return false;
  }
  if (!i2c_read_reg16(MAX17048_ADDR, 0x04, &soc)) {
    return false;
  }

  float voltage = (float)vcell * 0.000078125f;
  float percent = (float)soc / 256.0f;
  *out_voltage = voltage;
  *out_percent = percent;
  return true;
}

static bool read_lc709203f(float *out_voltage, float *out_percent) {
  uint16_t vcell = 0;
  uint16_t soc = 0;
  if (!i2c_read_reg16(LC709203F_ADDR, 0x09, &vcell)) {
    return false;
  }
  if (!i2c_read_reg16(LC709203F_ADDR, 0x0D, &soc)) {
    return false;
  }

  float voltage = (float)vcell * 0.001f;
  float percent = (float)soc;
  *out_voltage = voltage;
  *out_percent = percent;
  return true;
}

static void update_status(battery_monitor_t monitor, float voltage,
                          float percent, bool valid) {
  s_status.valid = valid;
  s_status.voltage_v = voltage;
  s_status.percent = percent;
  s_status.monitor = monitor;
  s_status.last_update_us = esp_timer_get_time();
}

bool battery_init(void) {
  if (s_inited) {
    return true;
  }

  memset(&s_status, 0, sizeof(s_status));

#ifndef ALPHALOC_BATTERY_I2C_PORT
#define ALPHALOC_BATTERY_I2C_PORT 0
#endif
#ifndef ALPHALOC_BATTERY_SDA_PIN
#error "ALPHALOC_BATTERY_SDA_PIN must be set via build_flags when battery monitor is enabled"
#endif
#ifndef ALPHALOC_BATTERY_SCL_PIN
#error "ALPHALOC_BATTERY_SCL_PIN must be set via build_flags when battery monitor is enabled"
#endif
#ifdef ALPHALOC_BATTERY_I2C_POWER_PIN
  gpio_config_t pwr_cfg = {
      .pin_bit_mask = 1ULL << ALPHALOC_BATTERY_I2C_POWER_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&pwr_cfg));
  ESP_ERROR_CHECK(gpio_set_level(ALPHALOC_BATTERY_I2C_POWER_PIN, 0));
#endif

  s_port = (i2c_port_t)ALPHALOC_BATTERY_I2C_PORT;
  i2c_config_t cfg = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = ALPHALOC_BATTERY_SDA_PIN,
      .scl_io_num = ALPHALOC_BATTERY_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  esp_err_t err = i2c_param_config(s_port, &cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    return false;
  }
  err = i2c_driver_install(s_port, I2C_MODE_MASTER, 0, 0, 0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    return false;
  }

  s_inited = true;
  return true;
}

bool battery_read_now(void) {
  if (!s_inited && !battery_init()) {
    return false;
  }

#ifdef ALPHALOC_BATTERY_I2C_POWER_PIN
  ESP_ERROR_CHECK(gpio_set_level(s_i2c_power_pin, 1));
  vTaskDelay(pdMS_TO_TICKS(5));
#endif

  float voltage = 0.0f;
  float percent = 0.0f;
  if (read_max17048(&voltage, &percent)) {
    update_status(BATTERY_MONITOR_MAX17048, voltage, percent, true);
#if ALPHALOC_VERBOSE
    ESP_LOGI(TAG, "Battery MAX17048: %.2fV %.0f%%", (double)voltage,
             (double)percent);
#endif
#ifdef ALPHALOC_BATTERY_I2C_POWER_PIN
    ESP_ERROR_CHECK(gpio_set_level(s_i2c_power_pin, 0));
#endif
    return true;
  }
  if (read_lc709203f(&voltage, &percent)) {
    update_status(BATTERY_MONITOR_LC709203F, voltage, percent, true);
#if ALPHALOC_VERBOSE
    ESP_LOGI(TAG, "Battery LC709203F: %.2fV %.0f%%", (double)voltage,
             (double)percent);
#endif
#ifdef ALPHALOC_BATTERY_I2C_POWER_PIN
    ESP_ERROR_CHECK(gpio_set_level(s_i2c_power_pin, 0));
#endif
    return true;
  }

  update_status(BATTERY_MONITOR_NONE, 0.0f, 0.0f, false);
#if ALPHALOC_VERBOSE
  ESP_LOGI(TAG, "Battery monitor not detected");
#endif
#ifdef ALPHALOC_BATTERY_I2C_POWER_PIN
  ESP_ERROR_CHECK(gpio_set_level(s_i2c_power_pin, 0));
#endif
  return false;
}

bool battery_get_status(battery_status_t *out) {
  if (!out) {
    return false;
  }
  *out = s_status;
  return s_status.valid;
}

#else

bool battery_init(void) { return false; }
bool battery_read_now(void) { return false; }
bool battery_get_status(battery_status_t *out) {
  if (out) {
    memset(out, 0, sizeof(*out));
  }
  return false;
}

#endif
