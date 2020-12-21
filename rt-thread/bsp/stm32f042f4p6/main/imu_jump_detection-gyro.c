#include "imu_jump_detection.h"

static const int ACC_DIM = 3;
static const int JUMP_WINDOW_SIZE = 5;
static int is_first_imu_frame = 1;
static int jump_window = 0;
static float threshold_gravity = 6.0;
static float accelerometer[3] = {0.0f, 0.0f, 0.0f};
static float threshold_gyro = 0.4; /// rad/s
static float gyro[3] = {0.0f, 0.0f, 0.0f};

void imu_jump_detection_init_default() {
    int i = 0;
    is_first_imu_frame = 1;
    for (i = 0; i < ACC_DIM; ++i) {
        accelerometer[i] = 0.0f;
        gyro[i] = 0.0f;
    }
}

void imu_jump_detection_init(float multiple_gravity) {
    if (multiple_gravity > 0.0f) {
        threshold_gravity = multiple_gravity;
    }
    imu_jump_detection_init_default();
}

void imu_gyro_jump_detection_init(float gyro_threshold) {
    if (gyro_threshold > 0.05) {
        threshold_gyro = gyro_threshold;
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
    int i = 0;
    float acc_float[3];
    for (i = 0; i < ACC_DIM; ++i) {
        acc_float[i] = acc[i]*32.0f*9.8f/65536.0f;
    }
    return imu_jump_detect_float(acc_float);
}

int imu_gyro_jump_detect_float(const float _gyro[]) {
    int i = 0, jump_occur = 0;
    const float alpha = 0.99999f;
    const float threshold = threshold_gyro;
    static float diff[3];
    float normal = 0.0f;

    if (is_first_imu_frame) {
        is_first_imu_frame = 0;
        for (i = 0; i < ACC_DIM; ++i) {
            gyro[i] = _gyro[i];
        }
        return jump_occur;
    }

    if (jump_window-- > 0) {
        return jump_occur;
    }

    normal = 0.0;
    for (i = 0; i < ACC_DIM; ++i) {
        diff[i] = _gyro[i] - gyro[i];
        normal += diff[i]*diff[i];
    }
    
    if (normal > threshold) {
        jump_occur = 1;
        jump_window = JUMP_WINDOW_SIZE;
    } else {
        for (i = 0; i < ACC_DIM; ++i) {
            gyro[i] = gyro[i] * alpha + (1.f - alpha) * _gyro[i];
        }
    }
    return jump_occur; 
}

int imu_gyro_jump_detect_int(const int16_t gyro[]) {
    int i = 0;
    float gyro_float[3];
    const double kDegree2Rad = 3.14159265358979323846 / 180.0;
    const int kGyroMeasRange = 2000 * 2;
    /// gyroscope measurement bit range, 16 bits
    const int kGyroMeasBitRange = 65536;

    for (i = 0; i < ACC_DIM; ++i) {
        gyro_float[i] = gyro[i] * kDegree2Rad * kGyroMeasRange / kGyroMeasBitRange;
    }
    return imu_gyro_jump_detect_float(gyro_float);
}
