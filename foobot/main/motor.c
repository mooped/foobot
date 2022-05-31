/* pwm example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_err.h>

#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"

#include "motor.h"

#define M1A_PIN 26
#define M1B_PIN 27
#define M2A_PIN 22
#define M2B_PIN 21

#define M1_EN_PIN  23
#define M2_EN_PIN  16

#define MOTOR_COUNT 2

static const char *TAG = "motor";

uint32_t control_pin_num[MOTOR_COUNT][2] = {
  { M1A_PIN, M1B_PIN },
  { M2A_PIN, M2B_PIN },
};

const uint32_t en_num[MOTOR_COUNT] = {
  M1_EN_PIN,
  M2_EN_PIN,
};

// duties table, real_duty = duties[x]/PERIOD
void motor_init(void)
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1<<M1_EN_PIN) | (1<<M1A_PIN) | (1<<M1B_PIN) | (1<<M2_EN_PIN) | (1<<M2A_PIN) | (1<<M2B_PIN);
  io_conf.pull_down_en = 1;
  io_conf.pull_up_en = 0;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void motor_set(int speed, int channel)
{
  // Coast
  if (speed == MOTOR_COAST)
  {
    gpio_set_level(en_num[channel], 0);
    gpio_set_level(control_pin_num[channel][0], 0);
    gpio_set_level(control_pin_num[channel][1], 0);
  }
  // Drive
  else
  {
    gpio_set_level(en_num[channel], 1);
    if (speed > 0)
    {
      gpio_set_level(control_pin_num[channel][0], 0);
      gpio_set_level(control_pin_num[channel][1], 1);
    }
    else
    {
      gpio_set_level(control_pin_num[channel][0], 1);
      gpio_set_level(control_pin_num[channel][1], 0);
    }
  }
}

void motor_set_all(int m1, int m2)
{
  motor_set(m1, 0);
  motor_set(m2, 1);
}

