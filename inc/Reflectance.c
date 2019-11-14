// Reflectance.c
// Provide functions to take measurements using a QTR-8RC reflectance
// sensor array (Pololu part number 961).  This works by outputting to
// the sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Student version of GPIO lab
// Daniel and Jonathan Valvano
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


// reflectance LED illuminate connected to P5.3
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "../inc/Clock.h"
#include "../inc/LaunchPad.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    // write this as part of Lab 6
    P5->SEL0 &= ~(1 << 3);
    P5->SEL1 &= ~(1 << 3);

    //Make the IR led output
    P5->DIR |= (1 << 3);

    //Set all pins on port 7 low
    P7->SEL0 = 0;
    P7->SEL1 = 0;

}

void LineSensorTest_Init(void){
    Reflectance_Init();

    //Copying this from LaunchPad.c
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
    P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &= ~0x07;     //    all LEDs off
}

// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t sleep_time)
{
    uint8_t data = 0;

    P5->OUT |= (1 << 3);

    P7->DIR = 0xFF;
    P7->OUT = 0xFF;

    Clock_Delay1us(10);

    P7->DIR = 0;
    Clock_Delay1us(sleep_time);


    data = P7->IN;

    P5->OUT &= ~(1 << 3);

    return data;
}



void LineSensorTest(uint32_t time){
    uint8_t sensor_data = Reflectance_Read(time);
    set_color(sensor_data);

}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    // write this as part of Lab 6
  return 0; // replace this line
}


/*
 *                                  LINE SENSOR
 *      LEFT                                                        RIGHT
 *      ----------------------------------------------------------------
 *      |   *       *       *       *       *       *       *       *   |
 *      ----------------------------------------------------------------
 *          -332    -237    -142    -47     47      142     237     332
 *
 *      These values represent the distance from the centor of the sensor in
 *      milimeters
 *      
 */

int32_t W [8] = {332, 237, 142, 47, -47, -142, -237, -332};
// Perform sensor integration
// Input: data is 8-bit result from line sensor
//
//        LEFT SIDR         RIGHT SIDE
//  Data: b7 b6 b5 b4 b3 b2 b1 b0
//
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data){
    int32_t sum = 0;
    int32_t ones = 0;
    //iterate through the bits of `data` and update sum and ones approprietly
    for(int ii = 0; ii < 8; ii++){
        if((data >> ii) & 0x1){
            sum += W[ii];
            ones++;
        }

    }

    //This is kind of a hack for my floor, its a weird white tile and its
    //triggering all of the sensors (or at least a majority (i think)) so it
    //thats the case I want the robot to think that we're lost so we can end up
    //going backwards
    if(ones >= 6){
        sum = 0;
    }
    //For the edge case of when both of the center sensors are triggered,
    //resulting in an average of 0
    else if(ones > 0 && sum == 0){
        sum = ones;
    }

    // d = sum(i=0..7, bi * wi) / sum(i=0..7, bi)
    return sum / ones;

}


// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    P5->OUT |= (1 << 3);

    P7->DIR = 0xFF;
    P7->OUT = 0xFF;

    Clock_Delay1us(10);

    P7->DIR = 0;

}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    uint8_t data = 0;

    data = P7->IN;

    P5->OUT &= ~(1 << 3);

    return data;
}


