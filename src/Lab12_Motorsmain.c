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

// Driver test
void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

//void configure_ADC()
//{
//    // Set reference voltage to 2.5V
//    REF_A->CTL0 |= 0x40;
//    //Turn the reference module on
//    REF_A->CTL0 |= 0x1;
//
//    // Set ADC14SC (start conversion) to 0, turn the ADC module on
//    // This also sets the clock predivider to 0 (bits 31-30)
//    // Also sets ths clock divider to 0 (bits 24-22)
//    // This also sets the clock source to MCLK (bits 21-19(
//    ADC14->CTL0 = 0x180030;
//    // Sets all the channel's sources to external by setting them 0 (bits 27-23)
//    ADC14->CTL1 &= ~(0xf800000);
//
//    // P5.5 = A0, P5.4 = A1, P5.3 = A2
//    // Need to set the SEL pins for all of these to b11 to select the ADC functionality
//    P5->SEL0 |= 0x38;
//    P5->SEL1 |= 0x38;
//
//    // Make these bits input
//    P5->DIR  |= ~(0x38);
//
//    // set the starting address to 0 for ADC
//    ADC14->CTL1 &= ~(0x1f0000);
//
//    // Configure the memory addresses for each of our channels
//    //
//    // Select our 2.5 VREF that we set up as our VREF, channel 0
//    ADC14->MCTL[0] = 0x100;
//    // Select our 2.5 VREF that we set up as our VREF, channel 1
//    ADC14->MCTL[1] = 0x101;
//    // Select our 2.5 VREF that we set up as our VREF, channel 2, set it as end of sequence
//    ADC14->MCTL[2] = 0x182;
//
//    // Enable the conversion for ADC
//    ADC14->CTL0 |= 0x2;
//
//}
//
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



void read_ADC(uint32_t buf [2])
{
    //uint32_t buf [2];
    while(ADC14->CTL0&0x00010000){};    // 1) wait for BUSY to be zero
    ADC14->CTL0 |= 0x00000001;          // 2) start single conversion
    while((ADC14->IFGR0&0x02) == 0){};  // 3) wait for ADC14IFG0
    buf[0] = ADC14->MEM[0];
    buf[1] = ADC14->MEM[1];
    //return buf;
}

#define RED 0x1
#define GREEN 0x2
#define BLUE 0x4

void main(){
    uint32_t n;
    uint32_t m;
    uint32_t o;

    uint32_t *a;
    uint32_t *b;
    uint32_t *c;

    Clock_Init48MHz(); // makes it 48 MHz
    //SysTick_Init();
    //LaunchPad_Init();   // buttons and LEDs
    //EUSCIA0_Init();     // initialize UART

    //Motor_Init();
    //P4->DIR = 0x0;

    //configure_ADC();
    ADC0_InitSWTriggerCh67();

    while (1){
        //read_ADC(buf);
        ADC_In67(&m, &n, &o);
        //m=(*a)&0x3fff;
        //n=(*b)&0x3fff;
    }

    //Drive_Motors(50, 1, 50, 1);

    //EUSCIA0_OutString("Hello World!");
    //n=EUSCIA0_InUDec();
}
