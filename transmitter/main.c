#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>

#define NULL ((void*)0)

#define LATCH_PIN   PB0
#define SHIFT_PIN   PB1
#define D0_PIN      PB3
#define D1_PIN      PB4

#define ATAD_PIN    PB2

#define DDR_LATCH   DDB0
#define DDR_SHIFT   DDB1
#define DDR_D0      DDB3
#define DDR_D1      DDB4

#define DDR_ATAD    DDB2

void SetupHardware(void)
{
  wdt_disable();

  // Configure port B direction and pull ups
  DDRB |= (1<<DDR_LATCH)|(1<<DDR_SHIFT)|(1<<DDR_ATAD);
  PORTB |= (1<<D0_PIN)|(1<<D1_PIN);

  // Timer 0 setup
  TCCR0A = (1<<WGM01);                          // Clear on Compare Match (CTC) mode
  TCCR0B = (1<<CS02)|(0<<CS01)|(0<<CS00);       // CLK/256 prescaler setting
  OCR0A = 7;                                   // Compare match register - time in increments of 1/16khz
  TIMSK0 = (0<<OCIE0B)|(1<<OCIE0A)|(0<<TOIE0);  // Interrupt on OCR0A match

  // Disable pin change interrupts
  GIMSK &= ~((1<<INT0)|(1<<PCIE));

  // Enable interrupts
  sei();
}

typedef enum
{
  eTS_IDLE = 0,
  eTS_WARMUP_ZEROES,
  eTS_WARMUP_ONES,
  eTS_MESSAGE,
  eTS_TRAILER,
} ETransmissionState;

void radio_out(uint8_t value)
{
  if (value)
  {
    PORTB |= (1<<ATAD_PIN);
  }
  else
  {
    PORTB &= ~(1<<ATAD_PIN);
  }
}

// Transmission shared state
static uint8_t      state = eTS_IDLE;
static uint8_t      transmitPhase = 0;
static uint8_t      transmitCounter = 0;

// State used by eTS_MESSAGE
static const char*  pszTransmitByte = NULL;
static uint8_t      transmitBit = 0x01;
static uint8_t      messageLength = 0;

void begin_state_warmup_zeroes(void)
{
  transmitPhase = 0;
  transmitCounter = 14;
  state = eTS_WARMUP_ZEROES;
}

void begin_state_warmup_ones(void)
{
  transmitPhase = 0;
  transmitCounter = 4;
  state = eTS_WARMUP_ONES;
}

void begin_state_message(void)
{
  transmitPhase = 0;
  transmitCounter = messageLength;
  state = eTS_MESSAGE;
}

void begin_state_trailer(uint8_t value)
{
  transmitCounter = 4;
  transmitPhase = value ? 1 : 0;
  state = eTS_TRAILER;
}

void begin_transmission(const char* pszMessage, uint8_t length)
{
  // Cancel any existing trannsmission
  state = eTS_IDLE;

  // Initialise the message
  pszTransmitByte = pszMessage;
  messageLength = length;
 
  // Begin sending the message next timer interrupt
  begin_state_warmup_zeroes();
}

ISR(TIM0_COMPA_vect, ISR_BLOCK)
{
  switch (state)
  {
    case eTS_IDLE:
    {
      radio_out(0);
    } break;
    case eTS_WARMUP_ZEROES:
    {
      radio_out(!transmitPhase);
      if (transmitPhase == 1)
      {
        transmitPhase = 0;
        if (--transmitCounter == 0)
        {
          begin_state_warmup_ones();
        }
      }
      else
      {
        transmitPhase = 1;
      }
    } break;
    case eTS_WARMUP_ONES:
    {
      radio_out(transmitPhase);
      if (transmitPhase == 1)
      {
        transmitPhase = 0;
        if (--transmitCounter == 0)
        {
          begin_state_message();
        }
      }
      else
      {
        transmitPhase = 1;
      }
    } break;
    case eTS_MESSAGE:
    {
      const uint8_t out = ((*pszTransmitByte) & transmitBit) ? transmitPhase : !transmitPhase;
      radio_out(out);
      if (transmitPhase == 1)
      {
        if (transmitBit == (1<<7))
        {
          ++pszTransmitByte;
          transmitBit = 0x01;
          if (--transmitCounter == 0)
          {
            begin_state_trailer(!out);
          }
        }
        else
        {
          transmitBit = transmitBit <<  1;
        }
        transmitPhase = 0;
      }
      else
      {
        transmitPhase = 1;
      }
    } break;
    case eTS_TRAILER:
    {
      radio_out(transmitPhase);
      if (--transmitCounter == 0)
      {
        state = eTS_IDLE;
      }
    } break;
    default:
    {
      radio_out(0);
      state = eTS_IDLE;
    } break;
  }
  return;
}

static char joypadTransmitBuffer[4] = {0};

uint16_t wheel = 0;

uint8_t inputs[2][2] = {
  {0, 0},
  {0, 0},
};

// Map robots to a controller channel, index and callsign
typedef struct
{
  uint8_t   channel;
  uint8_t   index;
  char      callsign;
} SRobotMap;

#define NUM_ROBOTS 4
static const SRobotMap robot_map[NUM_ROBOTS] = {
  { 0, 0, '1' },
  { 0, 1, 'a' },
  { 1, 0, '2' },
  { 1, 1, 'b' },
};

uint8_t current_robot = 0;

int main(void)
{
  SetupHardware();

  for (;;)
  {
    // If idle, process controller inputs and transmit appropriate messages
    if (state == eTS_IDLE)
    {
      // Serially read from 4 controllers (2 chains of 2 controllers)
      {
        PORTB |= (1<<LATCH_PIN);  // Enable the latch line
        _delay_us(12);
        PORTB &= ~(1<<LATCH_PIN);
        _delay_us(6);
  
        // Loop on controller index
        for (int controller_index = 0; controller_index < 2; ++controller_index)
        {
          inputs[0][controller_index] = 0;
          inputs[1][controller_index] = 0;

          // Read 8 bits from each line (read first, then send the next clock)
          for (int i = 0; i < 8; ++i)
          {
            inputs[0][controller_index] <<= 1;
            inputs[1][controller_index] <<= 1;
            inputs[0][controller_index] |= !(PINB & (1<<D0_PIN));
            inputs[1][controller_index] |= !(PINB & (1<<D1_PIN));

            PORTB |= (1<<SHIFT_PIN);
            _delay_us(6);
            PORTB &= ~(1<<SHIFT_PIN);
            _delay_us(6);
          }
        }

        // Transmit controller data for the current robot
        const SRobotMap* pRobot = &robot_map[current_robot];
        const uint8_t padstate = inputs[pRobot->channel][pRobot->index];
        
        joypadTransmitBuffer[0] = pRobot->callsign;
        joypadTransmitBuffer[1] = (padstate & 0x3f) | ((padstate >> 2) & 0x30); // Avoid using upper bits
        joypadTransmitBuffer[2] = ~(joypadTransmitBuffer[1]);
        joypadTransmitBuffer[3] = '\r';
        begin_transmission(joypadTransmitBuffer, 3);

        if (++current_robot >= NUM_ROBOTS)
        {
          current_robot = 0;
        }
      }
    }
    else
    {
      PORTB &= ~(1<<SHIFT_PIN);
    }
  }
}

