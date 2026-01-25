#ifndef ALPHALOC_NEOPIXEL_H
#define ALPHALOC_NEOPIXEL_H

#include <stdint.h>

void neopixel_init(int gpio_pin, uint8_t brightness);
void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#endif
