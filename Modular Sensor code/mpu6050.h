// mpu6050.h
#pragma once

typedef struct {
    double ax_g, ay_g, az_g;
    double gx_dps, gy_dps, gz_dps;
    double temp_c;
} mpu6050_sample_t;

int  mpu6050_open(const char *i2cdev);
void mpu6050_close(int fd);
int  mpu6050_init(int fd);
int  mpu6050_read_sample(int fd, mpu6050_sample_t *out);