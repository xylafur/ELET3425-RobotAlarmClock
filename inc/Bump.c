// Bump.c
// Runs on MSP432
// Provide low-level functions that interface bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 2, 2017

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

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

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
#include "../inc/PinManip.h"

#define NUM_BUMP_SENSORS 6
#define BUMP_SENSOR_PORT P4

//I wanted to make it dynamically grab the port as well but the port is some
//weird type, actually two weird types, so it would be a bit wonky.  Still
//opting for this because of less magic.  Though this does obviously force us
//to use the same port for all of the bump sensors
uint8_t bump_sensor_pins [NUM_BUMP_SENSORS] = {0, 1, 2, 3, 4, 5};

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    uint8_t ii;
    for(ii = 0; ii < NUM_BUMP_SENSORS; ii++){
        MK_GPIO_PULLUP(BUMP_SENSOR_PORT, bump_sensor_pins[ii]);
    }
}

/*  Read current state of 6 switches
 *  Returns a bitmask of which switches are on [5:0]
 */
uint8_t Bump_Read(void){
    uint8_t ii, data = 0;
    for(ii = 0; ii < NUM_BUMP_SENSORS; ii++){
        data |= (READ(BUMP_SENSOR_PORT, bump_sensor_pins[ii]));
    }
    
    return (~data) & 0x3f;
}

