#include "spi.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "rom/ets_sys.h"

static const char *TAG = "spi";

static gpio_num_t clock;
static gpio_num_t latch;
static gpio_num_t ch1;
static gpio_num_t ch2;

#define NUM_BITS 16

uint16_t ch1_buf = 0;
uint16_t ch2_buf = 0;

// Delegate to send commands to the main program
void process_command(uint8_t target, uint8_t command);

// Read SPI
void spi_read()
{
  int phase = 0;
  gpio_set_level(latch, 1);
  for (int bit = 0; bit < NUM_BITS;)
  {
    //ESP_LOGI(TAG, "Bit %d Phase %d", bit, phase);
    if (phase == 0)
    {
      if (bit > 0)
      {
        gpio_set_level(clock, 1);
      }

      phase = 1;
    }
    else
    {
      // Read this bit from both channels
      const uint16_t ch1_val = gpio_get_level(ch1);
      const uint16_t ch2_val = gpio_get_level(ch2);

      ch1_buf = (ch1_buf << 1) | ch1_val;
      ch2_buf = (ch2_buf << 1) | ch2_val;

      //ESP_LOGI(TAG, "Ch1: %x (%d) Ch2: %x (%d)", ch1_buf, ch1_val, ch2_buf, ch2_val);
   
      // Clear latch and clock
      gpio_set_level(latch, 0);
      gpio_set_level(clock, 0);
  
      phase = 0;
      ++bit;
    }

    ets_delay_us(60);
  }
  
  // Pass on the commands
  process_command('a', ~ch1_buf & 0xff);
  process_command('b', ~ch2_buf & 0xff);
  process_command('1', ~(ch1_buf & 0xff00) >> 8);
  process_command('2', ~(ch2_buf & 0xff00) >> 8);
}

void spi_init(gpio_num_t clock_, gpio_num_t latch_, gpio_num_t ch1_, gpio_num_t ch2_)
{
  clock = clock_;
  latch = latch_;
  ch1 = ch1_;
  ch2 = ch2_;

  // Configure inputs
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1 << ch1) | (1 << ch2);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  // Configure outputs
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1 << clock) | (1 << latch);
  io_conf.pull_down_en = 1;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  gpio_set_level(clock, 0);
  gpio_set_level(latch, 0);

  // Initialise timer
  /*
  const esp_timer_create_args_t periodic_timer_args =
  {
    .callback = &spi_timer_callback,
    .name = "periodic"
  };

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 6));
  */
}

