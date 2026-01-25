#include "neopixel.h"

#include <string.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
  rmt_encoder_t base;
  rmt_encoder_t *bytes_encoder;
  rmt_encoder_t *copy_encoder;
  int state;
  rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

static const char *TAG = "neopixel";
static rmt_channel_handle_t s_tx_chan;
static rmt_encoder_handle_t s_led_encoder;
static uint8_t s_brightness = 8;

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder,
                                   rmt_channel_handle_t channel,
                                   const void *primary_data,
                                   size_t data_size,
                                   rmt_encode_state_t *ret_state) {
  rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
  rmt_encode_state_t session_state = RMT_ENCODING_RESET;
  rmt_encode_state_t state = RMT_ENCODING_RESET;
  size_t encoded_symbols = 0;

  switch (led_encoder->state) {
    case 0:
      encoded_symbols += led_encoder->bytes_encoder->encode(
          led_encoder->bytes_encoder, channel, primary_data, data_size, &session_state);
      if (session_state & RMT_ENCODING_COMPLETE) {
        led_encoder->state = 1;
      }
      if (session_state & RMT_ENCODING_MEM_FULL) {
        state |= RMT_ENCODING_MEM_FULL;
        goto out;
      }
      // fall-through
    case 1:
      encoded_symbols += led_encoder->copy_encoder->encode(
          led_encoder->copy_encoder, channel, &led_encoder->reset_code,
          sizeof(led_encoder->reset_code), &session_state);
      if (session_state & RMT_ENCODING_COMPLETE) {
        led_encoder->state = RMT_ENCODING_RESET;
        state |= RMT_ENCODING_COMPLETE;
      }
      if (session_state & RMT_ENCODING_MEM_FULL) {
        state |= RMT_ENCODING_MEM_FULL;
        goto out;
      }
  }

out:
  *ret_state = state;
  return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder) {
  rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
  if (led_encoder->bytes_encoder) {
    rmt_del_encoder(led_encoder->bytes_encoder);
  }
  if (led_encoder->copy_encoder) {
    rmt_del_encoder(led_encoder->copy_encoder);
  }
  free(led_encoder);
  return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder) {
  rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
  rmt_encoder_reset(led_encoder->bytes_encoder);
  rmt_encoder_reset(led_encoder->copy_encoder);
  led_encoder->state = RMT_ENCODING_RESET;
  return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(uint32_t resolution_hz,
                                           rmt_encoder_handle_t *ret_encoder) {
  esp_err_t ret = ESP_OK;
  rmt_led_strip_encoder_t *led_encoder = NULL;
  ESP_GOTO_ON_FALSE(ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
  led_encoder = rmt_alloc_encoder_mem(sizeof(rmt_led_strip_encoder_t));
  ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for encoder");
  led_encoder->base.encode = rmt_encode_led_strip;
  led_encoder->base.del = rmt_del_led_strip_encoder;
  led_encoder->base.reset = rmt_led_strip_encoder_reset;

  rmt_bytes_encoder_config_t bytes_encoder_config = {
      .bit0 = {
          .level0 = 1,
          .duration0 = 0.3 * resolution_hz / 1000000,
          .level1 = 0,
          .duration1 = 0.9 * resolution_hz / 1000000,
      },
      .bit1 = {
          .level0 = 1,
          .duration0 = 0.9 * resolution_hz / 1000000,
          .level1 = 0,
          .duration1 = 0.3 * resolution_hz / 1000000,
      },
      .flags.msb_first = 1,
  };
  ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err,
                    TAG, "create bytes encoder failed");
  rmt_copy_encoder_config_t copy_encoder_config = {};
  ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err,
                    TAG, "create copy encoder failed");

  uint32_t reset_ticks = resolution_hz / 1000000 * 50 / 2;
  led_encoder->reset_code = (rmt_symbol_word_t){
      .level0 = 0,
      .duration0 = reset_ticks,
      .level1 = 0,
      .duration1 = reset_ticks,
  };
  *ret_encoder = &led_encoder->base;
  return ESP_OK;

err:
  if (led_encoder) {
    if (led_encoder->bytes_encoder) {
      rmt_del_encoder(led_encoder->bytes_encoder);
    }
    if (led_encoder->copy_encoder) {
      rmt_del_encoder(led_encoder->copy_encoder);
    }
    free(led_encoder);
  }
  return ret;
}

static void scale_rgb(uint8_t *r, uint8_t *g, uint8_t *b) {
  uint16_t scale = s_brightness;
  *r = (uint8_t)((*r * scale) / 255);
  *g = (uint8_t)((*g * scale) / 255);
  *b = (uint8_t)((*b * scale) / 255);
}

void neopixel_init(int gpio_pin, uint8_t brightness) {
  s_brightness = brightness;

  rmt_tx_channel_config_t tx_config = {
      .gpio_num = gpio_pin,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000,
      .mem_block_symbols = 64,
      .trans_queue_depth = 1,
      .intr_priority = 0,
  };
  tx_config.flags.with_dma = 0;
  tx_config.flags.invert_out = 0;

  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &s_tx_chan));
  ESP_ERROR_CHECK(rmt_new_led_strip_encoder(tx_config.resolution_hz, &s_led_encoder));
  ESP_ERROR_CHECK(rmt_enable(s_tx_chan));
}

void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (!s_tx_chan || !s_led_encoder) {
    return;
  }
  scale_rgb(&r, &g, &b);
  uint8_t grb[3] = {g, r, b};
  rmt_transmit_config_t tx_cfg = {
      .loop_count = 0,
  };
  tx_cfg.flags.eot_level = 0;
  tx_cfg.flags.queue_nonblocking = 0;
  ESP_ERROR_CHECK(rmt_transmit(s_tx_chan, s_led_encoder, grb, sizeof(grb), &tx_cfg));
  ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_chan, 20));
}
