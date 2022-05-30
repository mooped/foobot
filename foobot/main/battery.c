/*
 * Battery level sensor for FooBot
 *
 * Steve Barnett November 2018
 *
 */

#include "battery.h"

#include "esp_adc_cal.h"

#define ATTEN ADC_ATTEN_DB_0
#define WIDTH ADC_WIDTH_BIT_12

static adc1_channel_t battery_channel;

static esp_adc_cal_characteristics_t characteristics;
static esp_adc_cal_value_t calibration;

void battery_init(adc1_channel_t channel)
{
  battery_channel = channel;

  // Initialise ADC for the thermistor
  adc1_config_width(WIDTH);
  adc1_config_channel_atten(battery_channel, ATTEN);

  // Characterize ADC
  calibration = esp_adc_cal_characterize(ADC_UNIT_1, ATTEN, WIDTH, 0, &characteristics);
}

float battery_read_voltage(void)
{
  //uint32_t reading = adc1_get_raw(battery_channel);
  uint32_t millivolts = -1;
  if (esp_adc_cal_get_voltage(battery_channel, &characteristics, &millivolts) != ESP_OK)
  {
    return -1.f;
  }
  return (float)millivolts / 1000.f;
}

