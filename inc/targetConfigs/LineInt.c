#include <stdint.h>
#include "msp432.h"

#include "PinMap.h"
#include "PinManip.h"

#include "Clock.h"

#include "TimerUtil.h"
#include "TimerA2.h"
#include "CortexM.h"

#include "LineInt.h"

#include "../inc/LaunchPad.h"

/*  This module assumes that SM has already be initialized to 12 MHz
 */

#define LINE_SENSOR_PAUSE_MILI_SECONDS 10
#define CAP_CHARGE_TIME_MICRO_SECONDS 10
#define CAP_DISCHARGE_TIME_MICRO_SECONDS 1000

uint8_t LineSensorData = 0;


void LineSensorIntStart(void){
    long sr = StartCritical();

    //Turn on the line sensor LED
    HIGH(LINE_SENSOR_LED_PORT, LINE_SENSOR_LED_PIN);

    //Start charging the Capacitors on the line sensor
    MAKE_PORT_OUTPUT(LINE_SENSOR_SENSORS_PORT);
    WRITE_ONES_PORT(LINE_SENSOR_SENSORS_PORT);

    Clock_Delay1us(CAP_CHARGE_TIME_MICRO_SECONDS);

    MAKE_PORT_INPUT(LINE_SENSOR_SENSORS_PORT);

    TimerA2_Change_Task(LineSensorIntEnd,
                        MicroS_TO_PERIOD(CAP_DISCHARGE_TIME_MICRO_SECONDS));

    EndCritical(sr);
}

void LineSensorIntEnd(void){
    long sr = StartCritical();

    LineSensorData = READ_PORT(LINE_SENSOR_SENSORS_PORT);

    //turn off the IR LED
    LOW(LINE_SENSOR_LED_PORT, LINE_SENSOR_LED_PIN);

    EndCritical(sr);

    TimerA2_Change_Task(LineSensorIntStart,
                        MS_TO_PERIOD(LINE_SENSOR_PAUSE_MILI_SECONDS));
}


void LineSensorIntInit(void){
    //we will  be tunring this LED on and off
    MAKE_GPIO_OUTPUT(LINE_SENSOR_LED_PORT, LINE_SENSOR_LED_PIN);
    //we will be both reading and writing to / from this port;
    MAKE_PORT_GPIO(LINE_SENSOR_SENSORS_PORT);

    TimerA2_Init(LineSensorIntStart,
                 MS_TO_PERIOD(LINE_SENSOR_PAUSE_MILI_SECONDS));
}
