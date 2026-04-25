#define _POSIX_C_SOURCE 200809L
#include "mpu6050.h"

#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD_TO_DEG (180.0 / M_PI)
#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_XOUT  0x3B

static KalmanFilter g_roll_kf;
static KalmanFilter g_pitch_kf;
static int g_roll_kf_initialized = 0;
static int g_pitch_kf_initialized = 0;

static KalmanFilter *resolve_roll_kf(KalmanFilter *kf) {
    return kf ? kf : &g_roll_kf;
}

static KalmanFilter *resolve_pitch_kf(KalmanFilter *kf) {
    return kf ? kf : &g_pitch_kf;
}

static int i2c_write_reg(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return (write(fd, buf, 2) == 2) ? 0 : -1;
}

static int i2c_read_regs(int fd, uint8_t reg, uint8_t *buf, size_t n) {
    if (write(fd, &reg, 1) != 1) return -1;
    return (read(fd, buf, n) == (ssize_t)n) ? 0 : -1;
}

static int16_t be16(const uint8_t *b) {
    return (int16_t)((b[0] << 8) | b[1]);
}

int mpu6050_open(const char *i2cdev) {
    int fd = open(i2cdev, O_RDWR);
    if (fd < 0) return -1;
    if (ioctl(fd, I2C_SLAVE, MPU_ADDR) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

void mpu6050_close(int fd) {
    if (fd >= 0) close(fd);
}

int mpu6050_init(int fd) {
    return i2c_write_reg(fd, REG_PWR_MGMT_1, 0x00);
}

int mpu6050_read_sample(int fd, mpu6050_sample_t *out) {
    uint8_t b[14];
    int16_t ax, ay, az, t, gx, gy, gz;

    if (!out) return -1;
    if (i2c_read_regs(fd, REG_ACCEL_XOUT, b, sizeof(b)) != 0) return -1;

    ax = be16(&b[0]);
    ay = be16(&b[2]);
    az = be16(&b[4]);
    t  = be16(&b[6]);
    gx = be16(&b[8]);
    gy = be16(&b[10]);
    gz = be16(&b[12]);

    out->ax_g = (double)ax / 16384.0;
    out->ay_g = (double)ay / 16384.0;
    out->az_g = (double)az / 16384.0;

    out->gx_dps = (double)gx / 131.0;
    out->gy_dps = (double)gy / 131.0;
    out->gz_dps = (double)gz / 131.0;

    out->temp_c = ((double)t / 340.0) + 36.53;
    return 0;
}

void kalman_init(KalmanFilter *kf) {
    if (!kf) return;

    kf->angle = 0.0f;
    kf->rate  = 0.0f;
    kf->bias  = 0.0f;

    kf->Q_angle = 0.001f;
    kf->Q_bias  = 0.003f;
    kf->R_meas  = 0.03f;

    kf->P[0][0] = 10.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 10.0f;
}

void kalman_set_angle(KalmanFilter *kf, float angle) {
    if (!kf) return;
    kf->angle = angle;
}

float kalman_get_angle(KalmanFilter *kf, float raw_angle, float raw_rate, float dt) {
    float S, K0, K1, y;
    float P00_temp, P01_temp;

    if (!kf) return raw_angle;

    kf->rate = raw_rate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += dt * kf->Q_bias;

    S  = kf->P[0][0] + kf->R_meas;
    K0 = kf->P[0][0] / S;
    K1 = kf->P[1][0] / S;

    y = raw_angle - kf->angle;
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    P00_temp = kf->P[0][0];
    P01_temp = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    return kf->angle;
}

float kalman_get_rate(const KalmanFilter *kf) {
    if (!kf) return 0.0f;
    return kf->rate;
}

void kalman_set_Q_angle(KalmanFilter *kf, float v) { if (kf) kf->Q_angle = v; }
void kalman_set_Q_bias (KalmanFilter *kf, float v) { if (kf) kf->Q_bias  = v; }
void kalman_set_R_meas (KalmanFilter *kf, float v) { if (kf) kf->R_meas  = v; }
float kalman_get_Q_angle(const KalmanFilter *kf)   { return kf ? kf->Q_angle : 0.0f; }
float kalman_get_Q_bias (const KalmanFilter *kf)   { return kf ? kf->Q_bias  : 0.0f; }
float kalman_get_R_meas (const KalmanFilter *kf)   { return kf ? kf->R_meas  : 0.0f; }

double mpu6050_accel_roll(double ax, double ay, double az) {
    (void)ax;
    return atan2(ay, az) * RAD_TO_DEG;
}

double mpu6050_accel_pitch(double ax, double ay, double az) {
    return atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}

void mpu6050_roll_init(KalmanFilter *kf, double ax, double ay, double az) {
    KalmanFilter *use_kf = resolve_roll_kf(kf);
    kalman_init(use_kf);
    kalman_set_angle(use_kf, (float)mpu6050_accel_roll(ax, ay, az));
    if (!kf) g_roll_kf_initialized = 1;
}

void mpu6050_pitch_init(KalmanFilter *kf, double ax, double ay, double az) {
    KalmanFilter *use_kf = resolve_pitch_kf(kf);
    kalman_init(use_kf);
    kalman_set_angle(use_kf, (float)mpu6050_accel_pitch(ax, ay, az));
    if (!kf) g_pitch_kf_initialized = 1;
}

double mpu6050_get_roll(KalmanFilter *kf,
                        double ax, double ay, double az,
                        double gyro_x, double dt) {
    KalmanFilter *use_kf = resolve_roll_kf(kf);
    double roll = mpu6050_accel_roll(ax, ay, az);
    float kal;

    if (!kf && !g_roll_kf_initialized) {
        mpu6050_roll_init(NULL, ax, ay, az);
    }

    kal = use_kf->angle;
    if ((roll < -90.0 && kal > 90.0f) || (roll > 90.0 && kal < -90.0f)) {
        kalman_set_angle(use_kf, (float)roll);
        return roll;
    }

    return (double)kalman_get_angle(use_kf, (float)roll, (float)gyro_x, (float)dt);
}

double mpu6050_get_pitch(KalmanFilter *kf,
                         double ax, double ay, double az,
                         double gyro_y, double dt) {
    KalmanFilter *use_kf = resolve_pitch_kf(kf);
    double pitch = mpu6050_accel_pitch(ax, ay, az);

    if (!kf && !g_pitch_kf_initialized) {
        mpu6050_pitch_init(NULL, ax, ay, az);
    }

    return (double)kalman_get_angle(use_kf, (float)pitch, (float)gyro_y, (float)dt);
}

void mpu6050_attitude_init(mpu6050_attitude_filter_t *filters,
                           double ax, double ay, double az) {
    if (!filters) {
        mpu6050_roll_init(NULL, ax, ay, az);
        mpu6050_pitch_init(NULL, ax, ay, az);
        return;
    }

    mpu6050_roll_init(&filters->roll, ax, ay, az);
    mpu6050_pitch_init(&filters->pitch, ax, ay, az);
}

mpu6050_attitude_t mpu6050_get_attitude(mpu6050_attitude_filter_t *filters,
                                        double ax, double ay, double az,
                                        double gyro_x, double gyro_y,
                                        double dt) {
    mpu6050_attitude_t out;

    if (filters) {
        out.roll_deg = mpu6050_get_roll(&filters->roll, ax, ay, az, gyro_x, dt);
        out.pitch_deg = mpu6050_get_pitch(&filters->pitch, ax, ay, az, gyro_y, dt);
    } else {
        out.roll_deg = mpu6050_get_roll(NULL, ax, ay, az, gyro_x, dt);
        out.pitch_deg = mpu6050_get_pitch(NULL, ax, ay, az, gyro_y, dt);
    }

    return out;
}
