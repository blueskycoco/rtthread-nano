#include "imu_jump_detection.h"

static const int ACC_DIM = 3;
static const int JUMP_WINDOW_SIZE = 5;
static float threshold_gravity = 6.0;
static int is_first_imu_frame = 1;
static int jump_window = 0;
static float accelerometer[3] = {0.0f, 0.0f, 0.0f};

void imu_jump_detection_init_default() {
    int i = 0;
    is_first_imu_frame = 1;
    for (i = 0; i < ACC_DIM; ++i) {
        accelerometer[i] = 0.0f;
    }
}

void imu_jump_detection_init(float multiple_gravity) {
    if (multiple_gravity > 0.0f) {
        threshold_gravity = multiple_gravity;
    }
    imu_jump_detection_init_default();
}

int imu_jump_detect_float(const float acc[]) {
    int i = 0, jump_occur = 0;
    const float alpha = 0.99999f;
    const float acc_threshold = threshold_gravity*threshold_gravity*9.8f*9.8f;
    static float acc_diff[3];
    float acc_normal = 0.0f;

    if (is_first_imu_frame) {
        is_first_imu_frame = 0;
        for (i = 0; i < ACC_DIM; ++i) {
            accelerometer[i] = acc[i];
        }
        return jump_occur;
    }

    if (jump_window-- > 0) {
        return jump_occur;
    }

    acc_normal = 0.0;
    for (i = 0; i < ACC_DIM; ++i) {
        acc_diff[i] = acc[i] - accelerometer[i];
        acc_normal += acc_diff[i]*acc_diff[i];
    }

    if (acc_normal > acc_threshold) {
        jump_occur = 1;
        jump_window = JUMP_WINDOW_SIZE;
    } else {
        for (i = 0; i < ACC_DIM; ++i) {
            accelerometer[i] = accelerometer[i] * alpha + (1.f - alpha) * acc[i];
        }
    }
    return jump_occur; 
}

int imu_jump_detect_int(const int16_t acc[]) {
    float acc_float[3];
	int i;
    for (i = 0; i < ACC_DIM; ++i) {
        acc_float[i] = acc[i]*32.0f*9.8f/65536.0f;
    }
    return imu_jump_detect_float(acc_float);
}