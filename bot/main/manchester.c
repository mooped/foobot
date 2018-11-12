#include "manchester.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "manchester";

/*
volatile uint32_t counter = 0;
volatile uint32_t cycles = 0;
volatile uint32_t counter_buffer[32];
volatile uint32_t state_buffer[32];
*/

// Delegate for incoming commands
extern void process_command(uint8_t target, uint8_t command);

// Foobot Manchester receiver high level

static char message[3];
static uint8_t message_byte = 0;

void on_message_start(void)
{
  message_byte = 0;
  message[0] = 0;
  message[1] = 0;
  message[2] = 0;
}

static int dbg = 0;

void on_byte_recieved(char data)
{
  gpio_set_level(22, (dbg++) % 2);
  message[message_byte++] = data;
  if (message_byte == 3)
  {
    message_byte = 0;
    // Byte 2 should be the inverse of byte 1
    if (message[1] == (message[2] ^ 0xff))
    {
      process_command(message[0], message[1]);
    }
    else
    {
      //ESP_LOGI(TAG, "Checksum error");
    }
  }
}

// Foobot manchester receiver low level

#define CYCLE_T 5
#define TOLERANCE 1

#define CAPTURE_BITS 8

typedef enum
{
  eRS_WAITING = 0,
  eRS_SYNC,
  eRS_START_BITS,
  eRS_MESSAGE,
} ERecieverState;

uint8_t state = eRS_WAITING;                      // Start off waiting for sync pulses
uint8_t reciever_count = 0;
uint8_t incoming = 0;
uint8_t last_was_half = 0;
uint8_t capture = 0;
uint8_t bits = 0;
  
uint8_t status = 0;

inline uint8_t isT(uint8_t ticks)
{
  return ticks >= (CYCLE_T - TOLERANCE) && ticks <= (CYCLE_T + TOLERANCE);
}

inline uint8_t is2T(uint8_t ticks)
{
  return ticks >= ((2*CYCLE_T) - TOLERANCE) && ticks <= ((2*CYCLE_T) + TOLERANCE);
}

void begin_state_waiting(void)
{
  state = eRS_WAITING;
}

void begin_state_sync(void)
{
  reciever_count = 0;
  state = eRS_SYNC;
}

void begin_state_start_bits(void)
{
  reciever_count = 1;               // First 1 bit is handled in sync state
  state = eRS_START_BITS;
}

void begin_state_message(void)
{
  capture = 0;
  bits = 0;
  status = 0;
  state = eRS_MESSAGE;
  on_message_start();
}

inline void handle_error(void)
{
  begin_state_waiting();
}

inline void handle_bit(uint8_t bit)
{
  switch (state)
  {
    case eRS_SYNC:
    {
      if (bit == 1)
      {
        begin_state_start_bits();
      }
    } break;
    case eRS_START_BITS:
    {
      if (bit == 1)
      {
        if (++reciever_count == 3)
        {
          begin_state_message();
        }
      }
      else
      {
        handle_error();
      }
    } break;
    case eRS_MESSAGE:
    {
      capture = ((capture >> 1) & 0x7f) | (bit?0x80:0x00);
      if (++bits == CAPTURE_BITS)
      {
        on_byte_recieved(capture);
        status = 1;

        capture = 0;
        bits = 0;
      }
    } break;
  }
}

// Interrupt when input pin changes state
void pin_change(uint8_t count, uint8_t pin_state)
{
  // Capture current timer value then reset the timer
  //uint8_t count = TCNT0;
  //TCNT0 = 0x00;

  if (state == eRS_WAITING)
  {
    if (is2T(count))
    {
      last_was_half = 0;
      incoming = pin_state;
      begin_state_sync();
    }
  }
  else
  {
    if (last_was_half)
    {
      if (isT(count))
      {
        // Next bit = current bit
        handle_bit(incoming);
        last_was_half = 0;
      }
      else
      {
        // Error! Wait for a new message and try to sync
        handle_error();
      }
    }
    else
    {
      if (isT(count))
      {
        last_was_half = 1;
      }
      else if (is2T(count))
      {
        // Next bit = opposite of current bit
        incoming = (incoming?0:1);
        handle_bit(incoming);
      }
      else
      {
        // Error! Wait for a new message and try to sync
        handle_error();
      }
    }

    /*
	  counter_buffer[counter] = count;
	  state_buffer[counter] = state;
	  if (++counter >= 32)
	  {
	    counter = 0;
	    ++cycles;
	  }
    */
  }
}

// ESP32 specific code
gpio_num_t pin;
uint8_t pin_value = 0;

uint32_t timer_val = 0;

void manchester_interrupt(void* pArg)
{
  static uint32_t last_timer_val = 0;
  uint8_t new_pin_value = gpio_get_level(pin);
  if (new_pin_value != pin_value)
  {
    pin_change(timer_val - last_timer_val, new_pin_value);
    last_timer_val = timer_val;
    pin_value = new_pin_value;
  }
}

void manchester_timer_callback(void* pArg)
{
  if (timer_val == 0xffffffff)
  {
    timer_val = 0;
  }
  else
  {
    ++timer_val;
  }
}

void manchester_init(gpio_num_t manchester_pin)
{
  // Assign pin
  pin = manchester_pin;

  // Configure input
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1 << pin);
  io_conf.pull_down_en = 1;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  // Configure debug output
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1 << 22);
  io_conf.pull_down_en = 1;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  gpio_set_level(22, 1);

  // Initialise timer
  const esp_timer_create_args_t periodic_timer_args =
  {
    .callback = &manchester_timer_callback,
    .name = "periodic"
  };

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 6));

  // Register isr
  if (gpio_install_isr_service(0) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to install ISR service");
  }
  if (gpio_isr_handler_add(pin, &manchester_interrupt, (void*) 0) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add ISR handler");
  }
}

