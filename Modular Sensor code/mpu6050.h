// mpu6050.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double ax_g, ay_g, az_g;
    double gx_dps, gy_dps, gz_dps;
    double temp_c;
} mpu6050_sample_t;

typedef struct {
    float angle;
    float rate;
    float bias;

    float Q_angle;
    float Q_bias;
    float R_meas;

    float P[2][2];
} KalmanFilter;

typedef struct {
    KalmanFilter roll;
    KalmanFilter pitch;
} mpu6050_attitude_filter_t;

typedef struct {
    double roll_deg;
    double pitch_deg;
} mpu6050_attitude_t;

int  mpu6050_open(const char *i2cdev);
void mpu6050_close(int fd);
int  mpu6050_init(int fd);
int  mpu6050_read_sample(int fd, mpu6050_sample_t *out);

void kalman_init(KalmanFilter *kf);
void kalman_set_angle(KalmanFilter *kf, float angle);
float kalman_get_angle(KalmanFilter *kf, float raw_angle, float raw_rate, float dt);
float kalman_get_rate(const KalmanFilter *kf);

void  kalman_set_Q_angle(KalmanFilter *kf, float v);
void  kalman_set_Q_bias (KalmanFilter *kf, float v);
void  kalman_set_R_meas (KalmanFilter *kf, float v);
float kalman_get_Q_angle(const KalmanFilter *kf);
float kalman_get_Q_bias (const KalmanFilter *kf);
float kalman_get_R_meas (const KalmanFilter *kf);

double mpu6050_accel_roll(double ax, double ay, double az);
double mpu6050_accel_pitch(double ax, double ay, double az);

void mpu6050_roll_init(KalmanFilter *kf, double ax, double ay, double az);
void mpu6050_pitch_init(KalmanFilter *kf, double ax, double ay, double az);

double mpu6050_get_roll(KalmanFilter *kf,
                        double ax, double ay, double az,
                        double gyro_x, double dt);

double mpu6050_get_pitch(KalmanFilter *kf,
                         double ax, double ay, double az,
                         double gyro_y, double dt);

void mpu6050_attitude_init(mpu6050_attitude_filter_t *filters,
                           double ax, double ay, double az);

mpu6050_attitude_t mpu6050_get_attitude(mpu6050_attitude_filter_t *filters,
                                        double ax, double ay, double az,
                                        double gyro_x, double gyro_y,
                                        double dt);

#ifdef __cplusplus
}
#endif
