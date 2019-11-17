#ifndef SEVEN_SEGMENT_HEADER
#define SEVEN_SEGMENT_HEADER

#include <stdint.h>
/*  SEGMENT0    SEGMENT1    SEGMENT2    SEGMENT3
 *
 *  segment 0 is the left most 7 segemnt display
 */
enum segment_e {
    SEGMENT_0 = 0,
    SEGMENT_1,
    SEGMENT_2,
    SEGMENT_3,

    NUM_SEGMENTS
} segment_e;

void seven_segment_setup();

void set_segment_value(enum segment_e segment, uint8_t val);
void write_to_segment(enum segment_e segment, uint8_t val);

#endif
