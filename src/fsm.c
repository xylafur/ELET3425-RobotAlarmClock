#include "fsm.h"

// Object distance is from 50 to 800 mm
#define OBJECT_THRESHOLD 200
enum transitions_e distance_to_obj(uint32_t right_distance, uint32_t center_distance, uint32_t left_distance)
{
    if(center_distance <= OBJECT_THRESHOLD){
        if(right_distance <= OBJECT_THRESHOLD){
            if(left_distance <= OBJECT_THRESHOLD){
                return OBJECT_CENTER;
            }
            return OBJECT_CENTER_RIGHT;
        }

        if(left_distance <= OBJECT_THRESHOLD){
            return OBJECT_CENTER_LEFT;
        }

        return OBJECT_CENTER;
    }

    if(right_distance <= OBJECT_THRESHOLD){
        return OBJECT_RIGHT;
    }

    if(left_distance <= OBJECT_THRESHOLD){
        return OBJECT_LEFT;
    }

    return OBJECT_NONE;
}

#define RED 0x1
#define GREEN 0x2
#define BLUE 0x4

#define PURPLE (RED|BLUE)
#define YELLOW (RED|GREEN)
#define CYAN   (BLUE|GREEN)
#define WHITE  (RED|BLUE|GREEN)

#define MOTOR_MAX_SPEED 50
#define MOTOR_MIN_SPEED 20

struct FSM_State_s FSM_States [NUM_STATES] = {
    {ROTATING_LEFT, MOTOR_MIN_SPEED, 0, MOTOR_MIN_SPEED, 1, PURPLE,
        {ROTATING_LEFT, ROTATING_LEFT, ROTATING_LEFT, ROTATING_LEFT, ROTATING_LEFT, MOVING_FORWARD}},

    {VEERING_LEFT, MOTOR_MIN_SPEED, 1, MOTOR_MAX_SPEED, 1, BLUE,
        {VEERING_RIGHT, ROTATING_RIGHT, ROTATING_LEFT, ROTATING_LEFT, VEERING_LEFT, MOVING_FORWARD}},

    {MOVING_FORWARD, MOTOR_MAX_SPEED, 1, MOTOR_MAX_SPEED, 1, RED,
        {VEERING_RIGHT, ROTATING_RIGHT, ROTATING_RIGHT, ROTATING_LEFT, VEERING_LEFT, MOVING_FORWARD}},

    {VEERING_RIGHT, MOTOR_MAX_SPEED, 1, MOTOR_MIN_SPEED, 1, GREEN,
        {VEERING_RIGHT, ROTATING_RIGHT, ROTATING_RIGHT, ROTATING_LEFT, VEERING_LEFT, MOVING_FORWARD}},

    {ROTATING_RIGHT, MOTOR_MAX_SPEED, 1, MOTOR_MIN_SPEED, 0, YELLOW,
        {ROTATING_RIGHT, ROTATING_RIGHT, ROTATING_RIGHT, ROTATING_RIGHT, ROTATING_RIGHT, MOVING_FORWARD}}
};


