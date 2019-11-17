#include "SevenSegment.h"
#include "msp.h"
#include "../inc/TimerA1.h"

void segment_update_task(void);

//Pin a should be MSB
const uint8_t number[10] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B};

// This is for the segment selection pins
#define SEGMENT_PORT P8
uint8_t segment_pin_map [NUM_SEGMENTS] = {5, 6, 7, 2};
uint8_t current_segment = SEGMENT_0, previous_segment;

#define LED_PORT P7
/*
enum LED_MAP {
    LED_A = 0x40,
    LED_B = 0x20,
    LED_C = 0x10,
    LED_D = 0x8,
    LED_E = 0x4,
    LED_F = 0x2,
    LED_G = 0x1
} LED_MAP_e;
*/
enum LED_MAP {
    LED_A = 0x80,
    LED_B = 0x40,
    LED_C = 0x20,
    LED_D = 0x10,
    LED_E = 0x8,
    LED_F = 0x4,
    LED_G = 0x2
} LED_MAP_e;
const uint8_t LED_MASK = LED_A | LED_B | LED_C | LED_D | LED_E | LED_F | LED_G;
#define LED_OFFSET (LED_G >> 1)

uint8_t segment_mapping [NUM_SEGMENTS];

// ------------------------ Setup ------------------------- //
void seven_segment_setup() {
  // Set pin directions  
  unsigned int ii;
  uint8_t tmp_mask;
  for(ii = 0; ii < NUM_SEGMENTS; ii++){
    tmp_mask = (1<<segment_pin_map[ii]);

    // Make each of the segment selction pins outputs
    SEGMENT_PORT->DIR |= tmp_mask;

    // Turn off alternate functions, we just want GPIO
    SEGMENT_PORT->SEL0 &= ~tmp_mask;
    SEGMENT_PORT->SEL1 &= ~tmp_mask;

    // Default to the display being off
    SEGMENT_PORT->OUT &= ~tmp_mask;

    // Set the intial values of the 7 segment, make them all 0
    segment_mapping[ii] = number[0];

  }
  // Set up leds
  LED_PORT->DIR |= LED_MASK;
  LED_PORT->SEL0 &= ~LED_MASK;
  LED_PORT->SEL1 &= ~LED_MASK;

  // Make the 7 segment update with a frequency of 2kHz
  // Timer A0 is set up with a clock frequency of 187 kHz.. this means we need to count ~93 times
  TimerA1_Init((*segment_update_task), 93);
}

void set_segment_value(enum segment_e segment, uint8_t val)
{
    uint8_t segment_mask, led_mask;
    if(val > 9){
        return;
    }
    if(segment >= NUM_SEGMENTS){
        return;
    }
    // the periodic update task will update the display
    segment_mapping[segment] = number[val];
}

void write_to_segment(enum segment_e segment, uint8_t val){
    uint8_t segment_mask, led_mask;
    if(val > 9){
        return;
    }
    if(segment >= NUM_SEGMENTS){
        return;
    }

    // not tuurning other off, assume that is done before this function is called
    segment_mask = 1 << segment_pin_map[segment];
    led_mask = number[val] << LED_OFFSET;

    SEGMENT_PORT->OUT = ~segment_mask;
    LED_PORT->OUT = led_mask;
}

// we only turn one of the displays on at once, but we need to toggle them fast enough so that the
// human eye cannot see it
void segment_update_task(void)
{
    // Turn off the old segment
    LED_PORT->OUT = 0;
    // We need to turn the next segment on
    current_segment = (current_segment + 1) % NUM_SEGMENTS;
    SEGMENT_PORT->OUT = ~(1<<segment_pin_map[current_segment]);
    // Turn on the correct LEDs, use whatever is in the global mapping array
    LED_PORT->OUT = segment_mapping[current_segment]<<(LED_OFFSET);
}
