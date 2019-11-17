#ifndef FSM_HEADER

#include <stdint.h>

enum transitions_e {
    OBJECT_LEFT = 0,
    OBJECT_CENTER_LEFT,
    OBJECT_CENTER,
    OBJECT_CENTER_RIGHT,
    OBJECT_RIGHT,
    OBJECT_NONE,

    NUM_TRANSITIONS
};

enum transitions_e distance_to_obj(uint32_t right_distance, uint32_t center_distance, uint32_t left_distance);

enum states_e {
    ROTATING_LEFT = 0,
    VEERING_LEFT,
    MOVING_FORWARD,
    VEERING_RIGHT,
    ROTATING_RIGHT,

    NUM_STATES
};

struct FSM_State_s {
    enum states_e state;

    uint16_t left_speed;    // As a percentage from 0-100
    uint16_t left_dir;      // Boolean value, 1 forward, 0 backwards

    uint16_t right_speed;   // As a percentage from 0-100
    uint16_t right_dir;     // Boolean value, 1 forward, 0 backwards
    uint32_t color;

    uint32_t next_state_index [NUM_TRANSITIONS];
} FSM_State_s ;




#define FSM_HEADER
#endif
