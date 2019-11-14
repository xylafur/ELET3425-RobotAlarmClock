#ifndef MOTOR_UTIL_HEADER
#define MOTOR_UTIL_HEADER

// After digging through the libs a bit, think this is where float is defined
#include <math.h>
#include "PinMap.h"
#include "PinManip.h"

#define RIGHT_WHEEL_FORWARD() LOW(RT_WHL_DIR_PORT, RT_WHL_DIR_PIN)
#define RIGHT_WHEEL_BACKWARD() HIGH(RT_WHL_DIR_PORT, RT_WHL_DIR_PIN)

#define RIGHT_WHEEL_ON() HIGH(RT_WHL_EN_PORT, RT_WHL_EN_PIN)
#define RIGHT_WHEEL_OFF() LOW(RT_WHL_EN_PORT, RT_WHL_EN_PIN)

#define LEFT_WHEEL_FORWARD() HIGH(LF_WHL_DIR_PORT, LF_WHL_DIR_PIN)
#define LEFT_WHEEL_BACKWARD() LOW(LF_WHL_DIR_PORT, LF_WHL_DIR_PIN)

#define LEFT_WHEEL_ON() HIGH(LF_WHL_EN_PORT, LF_WHL_EN_PIN)
#define LEFT_WHEEL_OFF() LOW(LF_WHL_EN_PORT, LF_WHL_EN_PIN)

#define WHEELS_LEFT() RIGHT_WHEEL_FORWARD(); LEFT_WHEEL_BACKWARD()
#define WHEELS_RIGHT()     RIGHT_WHEEL_BACKWARD(); LEFT_WHEEL_FORWARD()
#define WHEELS_FORWARD()     RIGHT_WHEEL_FORWARD(); LEFT_WHEEL_FORWARD()
#define WHEELS_BACKWARD()     RIGHT_WHEEL_BACKWARD(); LEFT_WHEEL_BACKWARD()

#define MAX(a, b) a > b ? a : b

#define PERIOD_ms 10

/*  - Makes all of the ports GPIO
 *  - Sets enable low, sets slp high
 *  - if PWM, then sets the Enable ports to be TImer A0 pins
 */
#define INIT_MOTORS(PWM)                                                \
                MAKE_GPIO_OUTPUT(RT_WHL_DIR_PORT, RT_WHL_DIR_PIN);      \
                MAKE_GPIO_OUTPUT(RT_WHL_EN_PORT, RT_WHL_EN_PIN);        \
                MAKE_GPIO_OUTPUT(RT_WHL_SLP_PORT, RT_WHL_SLP_PIN);      \
                LOW(RT_WHL_EN_PORT, RT_WHL_EN_PIN);                     \
                HIGH(RT_WHL_SLP_PORT, RT_WHL_SLP_PIN);                  \
                if(PWM){MAKE_OUTPUT(RT_WHL_EN_PORT, RT_WHL_EN_PIN);}    \
                                                                        \
                MAKE_GPIO_OUTPUT(LF_WHL_DIR_PORT, LF_WHL_DIR_PIN);      \
                MAKE_GPIO_OUTPUT(LF_WHL_EN_PORT, LF_WHL_EN_PIN);        \
                MAKE_GPIO_OUTPUT(LF_WHL_SLP_PORT, LF_WHL_SLP_PIN);      \
                LOW(LF_WHL_EN_PORT, LF_WHL_EN_PIN);                     \
                HIGH(LF_WHL_SLP_PORT, LF_WHL_SLP_PIN);                  \
                if(PWM){MAKE_OUTPUT(LF_WHL_EN_PORT, LF_WHL_EN_PIN);}

#endif
