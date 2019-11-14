// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Starter code for Lab 12, uses Systick software delay to create PWM
// Daniel Valvano
// July 7, 2017

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

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"
#include "../inc/MotorUtil.h"


void Motor_InitSimple(void){
  /*    The right motor is connected as follows:
   *        DIR     1.6
   *        PWM     2.6
   *        !SLP    3.6
   *    The left motor is connected as follows:
   *        DIR     1.7
   *        PWM     2.7
   *        !SLP    3.7
   *
   *    All of these are GPIO outputs
   */
  #define NUM_MOTORS 2
  uint8_t pins [NUM_MOTORS] = {6, 7};

  uint8_t ii;

  for(ii = 0 ; ii < NUM_MOTORS ; ii++){
    MAKE_GPIO_OUTPUT(P1, pins[ii]);
    MAKE_GPIO_OUTPUT(P2, pins[ii]);
    MAKE_GPIO_OUTPUT(P3, pins[ii]);
  }

  LOW(P2, 6);
  LOW(P2, 7);

  HIGH(P3, 6);
  HIGH(P3, 7);
}

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
  P1->OUT &= ~0xC0;
  P2->OUT &= ~0xC0;   // off
  P3->OUT &= ~0xC0;   // low current sleep mode
}

/*  I have these just because it makes everything seem less like magic
 */
void PWM(uint16_t high_time_us, uint16_t period_ms){
    uint16_t low_time_us = period_ms * 1000 - high_time_us;

    RIGHT_WHEEL_ON();
    LEFT_WHEEL_ON();

    SysTick_Wait1us(high_time_us);

    RIGHT_WHEEL_OFF();
    LEFT_WHEEL_OFF();

    SysTick_Wait1us(low_time_us);
}


void independent_pwm(uint16_t right_high_us, uint16_t left_high_us,
                     uint16_t period_ms, uint16_t granularity_us){
    RIGHT_WHEEL_ON();
    LEFT_WHEEL_ON();

    uint16_t max = MAX(right_high_us, left_high_us);

    uint16_t low_time_us = period_ms * 1000 - max;

    //bit 1 is left wheel, bit 0 is right wheel
    uint8_t spinning = 3;
    uint16_t time = 0;
    while(spinning){
        if(spinning & 1 && time >= right_high_us){
            RIGHT_WHEEL_OFF();
            spinning &= ~((uint8_t)1);
        }
        if(spinning & 2 && time >= left_high_us){
            LEFT_WHEEL_OFF();
            spinning &= ~((uint8_t)2);
        }
        //TODO: Calculate how much time is lost by these above instructions!!
        SysTick_Wait1us(granularity_us);
        time += granularity_us;
    }

    SysTick_Wait1us(low_time_us);
}

#define GRANULARITY_us 10
void spin_wheels_independent(uint16_t right_duty, uint16_t left_duty,
                             uint32_t time_ms, uint16_t period_ms){
    uint16_t right_high_us = right_duty * period_ms * 1000 / 100;
    uint16_t left_high_us = left_duty * period_ms * 1000 / 100;

    uint32_t t;
    for(t = 0; t < time_ms; t+= period_ms){
        independent_pwm(right_high_us, left_high_us,
                        period_ms, GRANULARITY_us);
        
    }
}

void soft_left(uint32_t time_ms){
    WHEELS_FORWARD();
    spin_wheels_independent(25, 15, time_ms, PERIOD_ms);
}

void hard_left(uint32_t time_ms){
    WHEELS_FORWARD();
    spin_wheels_independent(30, 13, time_ms, PERIOD_ms);
}

void really_hard_left(uint32_t time_ms){
    WHEELS_FORWARD();
    spin_wheels_independent(35, 10, time_ms, PERIOD_ms);
}


void soft_right(uint32_t time_ms){
    WHEELS_FORWARD();
    spin_wheels_independent(15, 25, time_ms, PERIOD_ms);
}

void hard_right(uint32_t time_ms){
    WHEELS_FORWARD();
    spin_wheels_independent(13, 30, time_ms, PERIOD_ms);
}

void really_hard_right(uint32_t time_ms){
    WHEELS_FORWARD();
    spin_wheels_independent(10, 35, time_ms, PERIOD_ms);
}

void spin(uint16_t duty, uint32_t time, uint16_t period){
    uint32_t t;
    for(t = 0; t < time ; t += period){
        PWM(duty, period);
        //TODO: Add in bumber stop logic
        //
    }
}

//Ramp up the wheels up to 'duty' over the time period 'time'
void ramp_up(uint16_t max_duty, uint32_t time, uint16_t period){
    uint32_t t;
    uint16_t duty;

    for(t = 0; t < time ; t += period){
        duty = (uint16_t)(((float)t / time) * max_duty);
        PWM(duty, period);
    }
}

void ramp_down(uint16_t initial_duty, uint32_t ramp_time, uint16_t period){
    uint32_t t;
    uint16_t duty;

    for(t = 0; t < ramp_time ; t += period){
        duty = (uint16_t)((1 - ((float)t / ramp_time)) * initial_duty);
        PWM(duty, period);
    }
}


void Motor_ForwardExtraSimple(){
    RIGHT_WHEEL_FORWARD();
    LEFT_WHEEL_FORWARD();

    RIGHT_WHEEL_ON();
    LEFT_WHEEL_ON();


    SysTick_Wait10us(10000);

    RIGHT_WHEEL_OFF();
    LEFT_WHEEL_OFF();
}



/*  Duty is the time in microseconds that the wave should be high
 *  Time is the time in 10ms that the pulse should happen
 */
void Motor_ForwardSimple(uint16_t duty, uint32_t time){
    //Set the direction to forward (0) for each motor
    WHEELS_FORWARD();
    spin(duty, time, PERIOD_ms);
}



/*  Ramp the wheels up to the duty cycle 'duty' over the period of time
 *  'ramp_time' and then continue spinning the wheels until 'run_time' has
 *  passed total. *
 *
 *  run_time and ramp_tim should both be in units of 10ms
 */
void Motor_Forward_With_Ramp(uint16_t duty, uint32_t run_time, uint32_t ramp_time){
    WHEELS_FORWARD();

    ramp_up(duty, ramp_time, PERIOD_ms);
    spin(duty, run_time - ramp_time, PERIOD_ms);
}
                             

void RampDown(uint16_t initial_duty, uint32_t ramp_time){
    ramp_down(initial_duty, ramp_time, PERIOD_ms);
}



void Motor_BackwardSimple(uint16_t duty, uint32_t time){
    WHEELS_BACKWARD();
    spin(duty, time, PERIOD_ms);
}

void Motor_Backward_With_Ramp(uint16_t duty, uint32_t run_time, uint32_t ramp_time){
    WHEELS_BACKWARD();

    ramp_up(duty, ramp_time, PERIOD_ms);
    spin(duty, run_time - ramp_time, PERIOD_ms);
}



void Motor_RightSimple(uint16_t duty, uint32_t time){
    WHEELS_RIGHT();
    spin(duty, time, PERIOD_ms);
}

void Motor_Ramp_Right(uint16_t duty, uint32_t run_time, uint32_t ramp_time){
    WHEELS_RIGHT();
    ramp_up(duty, ramp_time, PERIOD_ms);
    spin(duty, run_time - ramp_time, PERIOD_ms);
}
 

void Motor_LeftSimple(uint16_t duty, uint32_t time){
    WHEELS_LEFT();    
    spin(duty, time, PERIOD_ms);
}

void Motor_Ramp_Left(uint16_t duty, uint32_t run_time, uint32_t ramp_time){
    WHEELS_LEFT();

    ramp_up(duty, ramp_time, PERIOD_ms);
    spin(duty, run_time - ramp_time, PERIOD_ms);
}

void MoveForwardBack(){
    Motor_ForwardSimple(5000, 1000);

    SysTick_Wait10ms(100);

    Motor_BackwardSimple(5000, 1000);
}

void MoveSquare(){
    Motor_ForwardSimple(5000, 500);
    SysTick_Wait10ms(100);

    Motor_LeftSimple(5000, 110);
    SysTick_Wait10ms(100);

    Motor_ForwardSimple(5000, 500);
    SysTick_Wait10ms(100);

    Motor_LeftSimple(5000, 110);
    SysTick_Wait10ms(100);

    Motor_ForwardSimple(5000, 500);
}

void RampedSquare(){
    Motor_Forward_With_Ramp(5000, 1000, 500);
    RampDown(5000, 500);

    Motor_Ramp_Left(2500, 1000, 1000);
    RampDown(2500, 500);

    Motor_Forward_With_Ramp(5000, 1000, 500);
    RampDown(5000, 500);

    Motor_Ramp_Left(2500, 1000, 1000);
    RampDown(2500, 500);

    Motor_Forward_With_Ramp(5000, 1000, 500);
    RampDown(5000, 500);
}

void InverseRampedSquare(){
    Motor_Forward_With_Ramp(5000, 1000, 500);
    RampDown(5000, 500);

    Motor_Ramp_Right(2500, 400, 400);
    RampDown(2500, 100);

    Motor_Forward_With_Ramp(5000, 1000, 500);
    RampDown(5000, 500);

    Motor_Ramp_Right(2500, 400, 400);
    RampDown(2500, 100);

    Motor_Forward_With_Ramp(5000, 1000, 500);
    RampDown(5000, 500);
}

void FullRampTest(){
    Motor_Forward_With_Ramp(10000, 1500, 500);
    RampDown(10000, 500);
}


