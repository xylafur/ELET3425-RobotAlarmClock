#ifndef LINE_INT_HEADER
#define LINE_INT_HEADER


/*  Function that will initialize all of the pins for the Line Sensor then sets
 *  the TimerA2 to trigger LineSensorIntStart after LINE_SENSOR_PAUSE_MILI_SECONDS
 */
void LineSensorIntInit(void);

/*  Turns on the LED and starts charging the CAPs on the Line sensor
 *
 *  Schedules the LineSensorIntMiddle task for CAP_CHARGE_TIME_MICRO_SECONDS in
 *  the future
 */
void LineSensorIntStart(void);

/*  Makes the Caps (IR Leds hooked up to the caps?) inputs and then schedules
 *  the LineSensorIntEnd task for CAP_DISCHARGE_TIME_MICRO_SECONDS in the
 *  future
 */
void LineSensorIntMiddle(void);

/*  Reads in the Line Sensor data into the global variable LineSensorData and
 *  then schedules LineSensorIntStart for LINE_SENSOR_PAUSE_MILI_SECONDS in the
 *  future
 */
void LineSensorIntEnd(void);

#endif
