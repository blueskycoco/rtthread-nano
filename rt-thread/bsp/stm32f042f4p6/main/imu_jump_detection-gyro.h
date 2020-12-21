#ifndef IMU_JUMP_DETECTION_H
#define IMU_JUMP_DETECTION_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

// Use the default detection threshold
void imu_jump_detection_init_default();

// Set detection threshold: multiples of gravitational acceleration
void imu_jump_detection_init(float multiple_gravity);

// After the initialization is complete, call this function in a loop. When an
// impact is detected, this function returns 1, otherwise it returns 0
int imu_jump_detect_float(const float acc[]);

int imu_jump_detect_int(const int16_t acc[]);

// Set detection threshold: default 0.4rad/s
void imu_gyro_jump_detection_init(float gyro_threshold);

// After the initialization is complete, call this function in a loop. When an
// impact is detected, this function returns 1, otherwise it returns 0
int imu_gyro_jump_detect_float(const float gyro[]);

int imu_gyro_jump_detect_int(const int16_t gyro[]);



#ifdef __cplusplus
}
#endif

#endif // IMU_JUMP_DETECTION_H
