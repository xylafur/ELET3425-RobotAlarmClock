 // Lab12_Motorsmain.c
// Runs on MSP432
// Solution to Motors lab
// Daniel and Jonathan Valvano
// September 4, 2017

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


// Sever VCCMD=VREG jumper on Motor Driver and Power Distribution Board and connect VCCMD to 3.3V.
//   This makes P3.7 and P3.6 low power disables for motor drivers.  0 to sleep/stop.
// Sever nSLPL=nSLPR jumper.
//   This separates P3.7 and P3.6 allowing for independent control
// Left motor direction connected to P1.7 (J2.14)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P1.6 (J2.15)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)


#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/SysTick.h"
#include "../inc/LaunchPad.h"
#include "fsm.h"
#include "SevenSegment.h"

//#include "../inc/UART0.h"
//#include "../inc/EUSCIA0.h"

#include "../inc/Motor.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA2.h"
#include "../inc/Clock.h"


extern struct FSM_State_s FSM_States [NUM_STATES];

// Driver test
void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

uint32_t right_distance, center_distance, left_distance;

#define SENSOR_BUFFER_SIZE 3
uint32_t sensor_right, sensor_left, sensor_center;
uint32_t sensor_right_buff [SENSOR_BUFFER_SIZE];
uint32_t sensor_left_buff [SENSOR_BUFFER_SIZE];
uint32_t sensor_center_buff [SENSOR_BUFFER_SIZE];

uint32_t average_array(uint32_t * arr, uint32_t size)
{
    uint32_t avg = 0;
    int ii;
    for(ii = 0; ii < size; ii++){
        avg += arr[ii];
    }
    return (avg/size);
}
/*
 *  Dr= Ar/(nr + Br) + Cr
 *  Dc= Ac/(nc+ Bc) + Cr
 *  Dl= Al/(nl + Bl) + C
 */
// Constants for right
const uint32_t Ar = 1, Br = 1, Cr = 1;
// Constants for center
const uint32_t Ac = 1, Bc = 1, Cc = 1;
// Constants for left
const uint32_t Al = 1, Bl = 1, Cl = 1;

void convert_sensor_to_distance(uint32_t* right_distance, uint32_t* center_distance,
                                uint32_t* left_distance)
{
    //*right_distance = Ar / (sensor_right + Br) + Cr;
    //*center_distance = Ac / (sensor_center + Bc) + Cc;
    //*left_distance = Al / (sensor_left + Bl) + Cl;

    // TODO: This is temporary
    *right_distance  = sensor_right  >= 6000 ? 200 : 800;
    *center_distance = sensor_center >= 6000 ? 200 : 800;
    *left_distance   = sensor_left   >= 6000 ? 200 : 800;
}

void timerA2_task(void)
{
    int ii;
    for(ii = 0; ii < 3; ii++){
        ADC_In67(&(sensor_right_buff[ii]),
                 &(sensor_left_buff[ii]),
                 &(sensor_center_buff[ii]));
    }

    sensor_right = average_array(sensor_right_buff, SENSOR_BUFFER_SIZE);
    sensor_left = average_array(sensor_left_buff, SENSOR_BUFFER_SIZE);
    sensor_center = average_array(sensor_center_buff, SENSOR_BUFFER_SIZE);
    convert_sensor_to_distance(&right_distance, &center_distance, &left_distance);

}
#undef SENSOR_BUFFER_SIZE

#define BUTTON_PORT P4
const uint8_t UP_BUTTON_PIN = 0, DOWN_BUTTON_PIN = 1, START_BUTTON_PIN = 2;

void setup_buttons()
{
    uint8_t mask = ~((1<<UP_BUTTON_PIN) | (1<<DOWN_BUTTON_PIN) | (1<<START_BUTTON_PIN));
    BUTTON_PORT->DIR &= mask;
    BUTTON_PORT->SEL0 &= mask;
    BUTTON_PORT->SEL1 &= mask;

    BUTTON_PORT->REN |= ~mask;
    BUTTON_PORT->OUT &= mask;
}
inline uint8_t up_button()
{
    // Do a double not to make the value either 0 or 1
    return !!!((1<<UP_BUTTON_PIN) & BUTTON_PORT->IN);
}
inline uint8_t down_button()
{
    return !!!((1<<DOWN_BUTTON_PIN) & BUTTON_PORT->IN);
}
inline uint8_t start_button()
{
    return !!!((1<<START_BUTTON_PIN) & BUTTON_PORT->IN);
}

uint32_t get_timer_val()
{
    //return 10;
    uint32_t timer_val = 0;
    uint8_t val;
    while(1){
        seven_segment_display(timer_val);
        if(up_button()){
            timer_val++;
        }else if(down_button()){
            timer_val--;
        }else if(start_button()){
            break;
        }
    }
    return timer_val;
}

void count_down(uint32_t initial)
{
    while(initial > 0){
        seven_segment_display(initial--);
        Clock_Delay1ms(1000);
    }
    seven_segment_display(0);
}

void main(){
    struct FSM_State_s current_state = FSM_States[2];
    uint32_t next_state_index, transition;
    uint32_t countdown_val;

    Clock_Init48MHz(); // makes it 48 MHz
    SysTick_Init();
    LaunchPad_Init();   // buttons and LEDs

    // Set up PWM outputs for the motors
    Motor_Init();

    // Initialize our ADCs
    ADC0_InitSWTriggerCh67();
    // This will set up the periodic task for the 7 segment
    seven_segment_setup();
    setup_buttons();

    // This starts periodic polling for the sensors
    // The period is in units of 2 us, 38ms = 19000 counts
    TimerA2_Init(timerA2_task, 19000);

    while(1){
        countdown_val = get_timer_val();
        count_down(countdown_val);

        // Need to setup stop button edge triggered interrupt here

        while (1){
            // Figure out object position based on distances
            transition = distance_to_obj(right_distance, center_distance, left_distance);
            // Figure out the next state based on this states transition table
            next_state_index = current_state.next_state_index[transition];
            // Update the current state
            current_state = FSM_States[next_state_index];
            // Update LED output
            LaunchPad_Output(current_state.color);
            // Update motor speed and direction
            //Drive_Motors(current_state.left_speed, current_state.left_dir,
              //           current_state.right_speed, current_state.right_dir);
        }
    }
}
