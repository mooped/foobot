#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

void init_hardware(void)
{
  // Disable watchdog if enabled by bootloader/fuses
  MCUSR &= ~(1<<WDRF);
  wdt_disable();

  // Disable prescaler
  clock_prescale_set(clock_div_1);

  // Enable port A pin 0:1 as output
  DDRA = (1<<0)|(1<<1);
  PORTA = 0;

  // Enable port D pin 3 as input, pins 2 and 4:5 as outputs
  DDRD = (0<<PD3)|(1<<PD2)|(1<<PD4)|(1<<PD5);

  // Disable the motor driver initially
  PORTD &= ~(1<<PD2);

  // Enable interrupts on INT1 (PD3) on all state transitions
  MCUCR = (0<<ISC11)|(1<<ISC10);
  GIMSK |= (1<<INT1);

  // Setup timer 0
#if 1
  // Normal mode
  TCCR0A = 0;                                    // Normal mode (counting upwards from 0x00 to 0xff)
  TCCR0B = (1<<CS02)|(0<<CS01)|(0<<CS00);        // CLK/256 prescaler setting
  TIMSK = 0;                                     // No timer interrupts
  TCNT0 = 0;                                     // Start the timer from zero
#else
  // CTC mode for testing the timings
  TCCR0A = (1<<WGM01);                           // Clear on Compare Match (CTC) mode
  TCCR0B = (1<<CS02)|(0<<CS01)|(0<<CS00);        // CLK/256 prescaler setting
  OCR0A = 14;                                    // Compare match register - time in increments of 1/16khz
  TIMSK = (0<<OCIE0B)|(1<<OCIE0A)|(0<<TOIE0);    // Interrupt on OCR0A match
#endif

  // Enable UART for debugging
  UBRRH = 0;                                     // Set baud rate to 9600 (with 8mhz oscillator)
  UBRRL = 51;
  UCSRB = (0<<RXEN)|(1<<TXEN);                   // Enable only the transmitter
  UCSRC = (1<<USBS)|(3<<UCSZ0);                  // Frame format: 8data, 2stop

  sei();
}

void USART_Transmit(unsigned char data)
{
  while (!(UCSRA & (1<<UDRE)));                  // Wait for empty transmission buffer
  UDR = data;                                    // Send the data
}

#define BUTTON_A (1<<5)
#define BUTTON_B (1<<4)
#define BUTTON_U (1<<3)
#define BUTTON_D (1<<2)
#define BUTTON_L (1<<1)
#define BUTTON_R (1<<0)

#define MOTOR_A  ((1<<PA1)|(1<<PA0))
#define MOTOR_D  ((1<<PD5)|(1<<PD4))

volatile uint8_t pwm_threshold = 0;

#define IDLE_PWM 0x00
#define STRAIGHT_PWM 0x01
#define TURN_PWM 0x80
#define SPIN_PWM 0x60

void process_command(char cmd)
{
  uint8_t porta = PORTA & ~(MOTOR_A);
  uint8_t portd = PORTD & ~(MOTOR_D);

  uint8_t new_pwm_threshold = IDLE_PWM;

  if (cmd & BUTTON_B) // Forwards
  {
    if ((cmd & BUTTON_R) == 0)
    {
      porta |= (1<<PA0);
    }
    if ((cmd & BUTTON_L) == 0)
    {
      portd |= (1<<PD4);
    }
    new_pwm_threshold = (cmd & (BUTTON_L | BUTTON_R)) ? TURN_PWM : STRAIGHT_PWM;
  }
  else if (cmd & BUTTON_A)  // Reverse
  {
    if ((cmd & BUTTON_R) == 0)
    {
      porta |= (1<<PA1);
    }
    if ((cmd & BUTTON_L) == 0)
    {
      portd |= (1<<PD5);
    }
    new_pwm_threshold = (cmd & (BUTTON_L | BUTTON_R)) ? TURN_PWM : STRAIGHT_PWM;
  }
  else  // Spinning
  {
    if (cmd & BUTTON_L) // Spin left
    {
      porta |= (1<<PA0);
      portd |= (1<<PD5);
      new_pwm_threshold = SPIN_PWM;
    }
    else if (cmd & BUTTON_R)  // Spin right
    {
      porta |= (1<<PA1);
      portd |= (1<<PD4);
      new_pwm_threshold = SPIN_PWM;
    }
  }

  // Update the outputs as close together as possible
  pwm_threshold = new_pwm_threshold;
  PORTA = porta;
  PORTD = portd;
}

#define ROBOT_ID '1'

char message[3];
uint8_t message_byte = 0;

void on_message_start(void)
{
  message_byte = 0;
  message[0] = 0;
  message[1] = 0;
  message[2] = 0;
}

void on_byte_recieved(char data)
{
  message[message_byte++] = data;
  if (message_byte == 3)
  {
    message_byte = 0;
    if (message[0] == ROBOT_ID)  // For me
    {
      // Byte 2 should be the inverse of byte 1
      if (message[1] == (message[2] ^ 0xff))
      {
        process_command(message[1]);
      }
      else
      {
        USART_Transmit('C');
      }
      USART_Transmit(message[1]);
      USART_Transmit(message[2]);
    }
    else  // Not for me
    {
      USART_Transmit('N');
    }
  }
}

void set_pwm_pin(uint8_t enable)
{
  if (enable)
  {
    PORTD |= (1<<PD2);
  }
  else
  {
    PORTD &= ~(1<<PD2);
  }
}

const uint8_t pwm_max = 0xff;

uint8_t pwm_counter = 0;

void update_pwm(void)
{
  if (pwm_threshold >= 1)
  {
    if (++pwm_counter == pwm_max)
    {
      pwm_counter = 0;
    }

    if (pwm_counter < pwm_threshold)
    {
      set_pwm_pin(0);
    }
    else
    {
      set_pwm_pin(1);
    }
  }
  else
  {
    set_pwm_pin(0);
  }
}

int main(void)
{
  init_hardware();

  while (1)
  {
    update_pwm();
  }
}

#define CYCLE_T 7
#define TOLERANCE 2

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

// Interrupt when pin D3 changes state
ISR(INT1_vect, ISR_BLOCK)
{
  // Capture current timer value then reset the timer
  uint8_t count = TCNT0;
  TCNT0 = 0x00;

  if (state == eRS_WAITING)
  {
    if (is2T(count))
    {
      last_was_half = 0;
      incoming = (PIND>>PD3)&0x01;
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
  }
}

// Interrupt on timer 0 output compare A
ISR(TIMER0_COMPA_vect, ISR_BLOCK)
{
  //PORTA = ~PORTA;
}

