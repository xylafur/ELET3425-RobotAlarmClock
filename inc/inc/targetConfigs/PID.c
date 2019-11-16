

#include "../inc/FlashDebug.h"

#include <stdint.h>

#define MAX_BUFFER_SIZE 15
int16_t error_buffer [MAX_BUFFER_SIZE];
int16_t error_buffer_size = 0;
int16_t error_buffer_position = 0;

//                  b00011000
//          The middle two sensors, our goal position
#define COMMAND_VALUE 24

//This is both the max value and the minimum value that should come out of our
//control functions (p + i + d)
#define MAX_CONTROL_VALUE 65536
#define MIN_CONTROL_VALUE (-MAX_CONTROL_VALUE)

#define MAXIMUM_WHEEL_PERCENT 95
#define MINIMUM_WHEEL_PERCENT 10

float left_wheel_constant = 0;
float right_wheel_constant = 0;

void PID_init(){
    left_wheel_constant =   (MAXIMUM_WHEEL_PERCENT - MINIMUM_WHEEL_PERCENT) /
                            ((float)MAX_CONTROL_VALUE);
    right_wheel_constant =  (MINIMUM_WHEEL_PERCENT - MAXIMUM_WHEEL_PERCENT) /
                            ((float)MAX_CONTROL_VALUE);


    memset(error_buffer, 0, sizeof(int16_t) * MAX_BUFFER_SIZE);
}

#define PREVIOUS_ERROR (error_buffer[error_buffer_position == 0 ?           \
                                     error_buffer_size - 1 :                \
                                     error_buffer_position])


#define MAX_ERROR_VALUE (MAX_CONTROL_VALUE / 10)
#define MIN_ERROR_VALUE (MIN_CONTROL_VALUE / 10)

int8_t error_mapping [8] = {8, 4, 2, 1, -1, -2, -4, -8};

#define SYMETRIC(d) ((((d&0x80)>>7)==(d&0x1)) && (((d&0x40)>>5)==(d&0x2)) &&\
                             (((d&0x20)>>3)==(d&0x4)) && (((d&0x10)>>1)==(d&0x8)))

/*  This will return a range from MAX_ERROR_VALUE to MIN_ERROR_VALUE
 */
int32_t sensor_to_error(uint8_t sensor_data){
    //We're in the middle
    if(SYMETRIC(sensor_data)){
        return 0;
    }

    uint8_t ii, jj;
    int32_t sum = 0, count = 0;
    for(ii=0, jj=7; ii<8; jj--, ii++){
        if ((1<<jj) & sensor_data){
            sum += error_mapping[ii];
            count++;
        }
    }

    return MAX_ERROR_VALUE * sum / (count * 8);
}

#define ERROR error_buffer[(error_buffer_position-1) % MAX_BUFFER_SIZE]

void sensor_to_error_buffer(uint8_t sensor_data){
    error_buffer[error_buffer_position] = sensor_to_error(sensor_data);
    error_buffer_position = (error_buffer_position + 1) % MAX_BUFFER_SIZE;

    if(error_buffer_size < MAX_BUFFER_SIZE){
        error_buffer_size++;
    }
}



#define same_sign(a, b) ((a > 0 && b > 0) || (b < 0 && a < 0))

// TODO: Compute these values
//#define Kp
//#define Kd
//#define Ki
float Kp = 10.0;
float Kd = 10.0;
float Ki = 10.0;

void increaseKp(){
    Kp += 0.5;
}
void decreaseKp(){
    Kp -= 0.5;
}

uint8_t use_i = 1;

int32_t proportional(int32_t error){
    //int16_t error = error_buffer[error_buffer_position];
    return error;
}

/* Pure Naive Derivative
 */
int32_t derivative(){
    int16_t this_error = error_buffer[error_buffer_position],
            last_error = PREVIOUS_ERROR;

    return this_error - last_error;
}

int32_t integral(){
    int32_t sum = 0;
    uint8_t ii;
    for(ii = 0; error_buffer_size - ii > 0; ii++){
        sum += error_buffer[ii % MAX_BUFFER_SIZE];
    }
    return sum;
}

int32_t saturation_clamper(int32_t preClamp){
    if(preClamp > 0 && preClamp > MAX_CONTROL_VALUE){
        return MAX_CONTROL_VALUE;

    }else if(preClamp < 0 && preClamp < MIN_CONTROL_VALUE){
        return MIN_CONTROL_VALUE;
    }
    return preClamp;
}

#define RIGHT 1
#define LEFT 0
/*  X is expected to be a value between MIN_CONTROL_VALUE and MAX_CONTROL_VALUE
 */
uint16_t wheel_equation(uint8_t wheel, int32_t x){
    //right wheel
    if(wheel){
        return (uint16_t)(x * right_wheel_constant + MAXIMUM_WHEEL_PERCENT);
    //left wheel
    }else{
        return (uint16_t)(x * left_wheel_constant + MAXIMUM_WHEEL_PERCENT);
    }
}

/*  A control value of 0 means that there was no error and we should keep
 *  spinning out wheels at their maximum value (80%)
 *
 *  A positive control value means that the line is to the right of the robot,
 *  so we should decrease the speed of the right motor.
 *
 *  A negative control value means that the line is to the left of the robot
 *  and that the left wheel's speed should be decreased.
 *
 *  returns a bitmask of the two percentages for the wheels
 */
uint32_t control_to_action(int32_t control_value){
    uint16_t left_wheel_percent, right_wheel_percent;
    if(control_value == 0){
        left_wheel_percent = right_wheel_percent = MAXIMUM_WHEEL_PERCENT;

    } else if(control_value > 0){
        left_wheel_percent = MAXIMUM_WHEEL_PERCENT;
        right_wheel_percent = wheel_equation(RIGHT, control_value);

    }else if(control_value < 0){
        right_wheel_percent = MAXIMUM_WHEEL_PERCENT;
        left_wheel_percent = wheel_equation(LEFT, control_value);
    }

    return (left_wheel_percent << 16) | right_wheel_percent;
}

uint32_t compute_actuation(int32_t error){
    int32_t p = 0, i = 0, d = 0, preClamp = 0, postClamp = 0;
    //int16_t e = ERROR;

    buffer_write_flash_flush(0xbabe);
    buffer_write_flash_flush(0xcafe);
    buffer_write_flash_flush((error&0xFF00)>>16);
    buffer_write_flash_flush((error&0xFF));


    p = Kp * proportional(error);

    buffer_write_flash_flush(0xcafe);
    buffer_write_flash_flush(0xbabe);
    buffer_write_flash_flush((p&0xFF00)>>16);
    buffer_write_flash_flush((p&0xFF));
    buffer_write_flash_flush(0x1337);
    buffer_write_flash_flush(0xffff);
    /*
    if(use_i){
        i = Ki * integral();
    }

    if(error_buffer_size >= 2){
        d = Kd * derivative();
    }

    preClamp = p + i + d;


    if(preClamp != postClamp && same_sign(e, preClamp)){
        use_i = 0;
    }else{
        use_i = 1;
    }
    */
    preClamp = p;

    buffer_write_flash_flush(0xdead);
    buffer_write_flash_flush(0xbeef);

    postClamp = saturation_clamper(preClamp);

    buffer_write_flash_flush((postClamp&0xFF00)>>16);
    buffer_write_flash_flush((postClamp&0xFF));


    return control_to_action(postClamp);
}
