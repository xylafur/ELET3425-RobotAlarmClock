#ifndef PID_HEADER
#define PID_HEADER

#include <stdint.h>

void PID_init();
int32_t sensor_to_error(uint8_t sensor_data);
void sensor_to_error_buffer(uint8_t sensor_data);
void increaseKp();
void decreaseKp();
uint32_t compute_actuation(int32_t error);

#endif
