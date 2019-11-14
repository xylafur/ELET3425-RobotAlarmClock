/*  Simplified BSD License (FreeBSD License)
    Copyright (c) 2017, James Richardson, All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
    USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    The views and conclusions contained in the software and documentation are
    those of the authors and should not be interpreted as representing official
    policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>

#define AVG(a, b) ((a + b)/2)
#define BETWEEN(pos, max, min) (pos > min && pos < max)

/*  This way even though I will be changing the State Machine significantly, my
 *  old implementation (which works, just not that well and doesn't use
 *  interrupts) can still be compiled
 */
#ifdef LineFollowing1

/*  These represent all of the possible states of our Finite State Machine
 */
#define NUM_STATES 8
enum State_e {
    StateOffLeft,    //0
    StateFarLeft,    //1
    StateLeft,       //2
    StateCenter1,    //3
    StateCenter2,    //4
    StateRight,      //5
    StateFarRight,   //6
    StateOffRight,   //7
};


/* This enum represents all of the possible edges between each of the states.
 * Every state will have an outgoing transition coresponding to each of these
 * edges.
 *
 * If we get a reading of zero, it means that no sensors are triggered.  The
 * Position function is guaranteed to return a non zero number if there are at
 * least some sensors triggered (meaning that it is perfectly canceling out)
 *
 * This edges enum is just used to index the state's array.  Each state can
 * have different behavior for what to do if it gets a reading of 0
 */
#define NUM_EDGES 6
enum Edges {
    Off_e,
    FarLeft_e,
    Left_e,
    Center_e,
    Right_e,
    FarRight_e,

    Error_e
};

#define SLEEPTIME 500

/*  This stuct represents each state in our FSM.
 */
#define MAX_NAME_LEN 16
struct State {
    enum State_e this_state;
    uint32_t out;
    uint32_t delay;
    uint32_t drive_time;
    const struct State *next[NUM_EDGES];
};

typedef const struct State State_t;


void drive_motor(enum State_e this_state);
enum Edges edge_encoder(int32_t pos);

#endif




#ifdef LINEFOLLOWING2

enum State_e {
    OFF_LEFT_STATE,

    SUPER_HARD_LEFT_STATE,

    FAR_LEFT_STATE,
    LEFT_STATE,
    SLIGHTLY_LEFT_STATE,

    CENTER_FORWARD_STATE,
    CENTER_BACKWARD_STATE,

    SLIGHTLY_RIGHT_STATE,
    RIGHT_STATE,
    FAR_RIGHT_STATE,

    SUPER_HARD_RIGHT_STATE,

    OFF_RIGHT_STATE,

    //If the bump sensors are triggered.. just give up for now
    STALL_STATE,

    //Not a real state, but will give us the count of states
    NUM_STATES
};

enum Edges_e {
    FAR_LEFT_EDGE       = 0,
    LEFT_EDGE           = 1,
    SLIGHTLY_LEFT_EDGE  = 2,

    CENTER_EDGE         = 3,

    SLIGHTLY_RIGHT_EDGE = 4,
    RIGHT_EDGE          = 5,
    FAR_RIGHT_EDGE      = 6,

    //Generic edge for "we lost the line", it is up to the particular state to
    //decide how it moves from here
    OFF_EDGE            = 7,
    NUM_EDGES           = 7
};


struct State {
    enum State_e this_state;

    uint8_t color;

    enum State_e off_edge_transition;

    //Duties are expected to be in high time percentage
    uint16_t left_duty;
    uint8_t left_direction;

    uint16_t right_duty;
    uint8_t right_direction;
};

typedef struct State State_t;

enum Edges_e get_edge(int32_t pos);

#endif


State_t * get_starting_state();
