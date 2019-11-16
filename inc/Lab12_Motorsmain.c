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

//#include "../inc/UART0.h"
//#include "../inc/EUSCIA0.h"

#include "../inc/Motor.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA2.h"

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



enum transitions_e {
    OBJECT_LEFT = 0,
    OBJECT_CENTER_LEFT,
    OBJECT_CENTER,
    OBJECT_CENTER_RIGHT,
    OBJECT_RIGHT,
    OBJECT_NONE,

    NUM_TRANSITIONS
};

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

enum states_e {
    ROTATING_LEFT = 0,
    VEERING_LEFT,
    MOVING_FORWARD,
    VEERING_RIGHT,
    ROTATING_RIGHT,

    NUM_STATES
};

#define RED 0x1
#define GREEN 0x2
#define BLUE 0x4

#define PURPLE (RED|BLUE)
#define YELLOW (RED|GREEN)
#define CYAN   (BLUE|GREEN)
#define WHITE  (RED|BLUE|GREEN)


struct FSM_State_s {
    enum states_e state;

    uint16_t left_speed;    // As a percentage from 0-100
    uint16_t left_dir;      // Boolean value, 1 forward, 0 backwards

    uint16_t right_speed;   // As a percentage from 0-100
    uint16_t right_dir;     // Boolean value, 1 forward, 0 backwards
    uint32_t color;

    uint32_t next_state_index [NUM_TRANSITIONS];
} FSM_State_s ;

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

void configure_ADC()
{
    ADC14->CTL0 &= ~0x00000002;        // 2) ADC14ENC = 0 to allow programming
    while(ADC14->CTL0&0x00010000){};   // 3) wait for BUSY to be zero
    ADC14->CTL0 = 0x4223390;          // 4) single, SMCLK, on, disabled, /1, 32 SHM
    // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
    // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
    // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
    // 25    ADC14ISSH  invert sample-and-hold  0b =  not inverted
    // 24-22 ADC14DIVx  clock divider         000b = /1
    // 21-19 ADC14SSELx clock source select   100b = SMCLK
    // 18-17 ADC14CONSEQx mode select          01b = Sequence of channels
    // 16    ADC14BUSY  ADC14 busy              0b (read only)
    // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
    // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
    // 7     ADC14MSC   multiple sample         1b = not multiple
    // 6-5   reserved                          00b (reserved)
    // 4     ADC14ON    ADC14 on                1b = powered up
    // 3-2   reserved                          00b (reserved)
    // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
    // 0     ADC14SC    ADC14 start             0b = No start (yet)

    ADC14->CTL1 = 0x00000030;          // 5) ADC14MEM0, 14-bit, ref on, regular power
    // 20-16 STARTADDx  start addr          00000b = ADC14MEM0
    // 15-6  reserved                  0000000000b (reserved)
    // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
    // 3     ADC14DF    data read-back format   0b = Binary unsigned
    // 2     REFBURST   reference buffer burst  0b = reference on continuously
    // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode

    ADC14->MCTL[0] = 0x00000000;         // 6) 0 to 3.3V, channel 0
    // 15   ADC14WINCTH Window comp threshold   0b = not used
    // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
    // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
    // 12   reserved                            0b (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         0b = End of sequence
    // 6-5  reserved                           00b (reserved)
    // 4-0  ADC14INCHx  Input channel        0000b = A0, P5.5

    ADC14->MCTL[1] = 0x00000081;         // 6) 0 to 3.3V, channel 1
    // 15   ADC14WINCTH Window comp threshold   0b = not used
    // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
    // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
    // 12   reserved                            0b (reserved)
    // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
    // 7    ADC14EOS    End of sequence         1b = End of sequence
    // 6-5  reserved                           00b (reserved)
    // 4-0  ADC14INCHx  Input channel        0001b = A1, P5.4



    ADC14->IER0 = 0; // 7) no interrupts
    ADC14->IER1 = 0; // no interrupts
    P5->SEL0 |= 0x38;
    P5->SEL1 |= 0x38;

    ADC14->CTL0 |= 0x00000002;         // 9) enable
}

void main(){
    uint32_t a, b, c;
    Clock_Init48MHz(); // makes it 48 MHz
    SysTick_Init();
    LaunchPad_Init();   // buttons and LEDs

    Motor_Init();

    ADC0_InitSWTriggerCh67();

    // The period is in units of 2 us, 38ms = 19000 counts
    TimerA2_Init(timerA2_task, 19000);

    // Start moving forward
    struct FSM_State_s current_state = FSM_States[2], next_state;
    uint32_t next_state_index;

    while (1){
        uint32_t transition = distance_to_obj(right_distance, center_distance, left_distance);
        next_state_index = current_state.next_state_index[transition];

        current_state = FSM_States[next_state_index];
        LaunchPad_Output(current_state.color);
        Drive_Motors(current_state.left_speed, current_state.left_dir,
                     current_state.right_speed, current_state.right_dir);

    }
}
