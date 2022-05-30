/*
 * LED library for ESP datalogger
 * 
 * Steve Barnett 2018
 *
*/

#include "driver/gpio.h"

#include "led.h"

#define LED_PIN 25

void led_init(void)
{
    // Configure output pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << LED_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Turn LED off
    led_set(0);
}

void led_set(uint8_t state)
{
  gpio_set_level(LED_PIN, state);
}

