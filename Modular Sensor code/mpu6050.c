// mpu6050.c
#define _POSIX_C_SOURCE 200809L
#include "mpu6050.h"

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_XOUT  0x3B

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
    if (ioctl(fd, I2C_SLAVE, MPU_ADDR) < 0) { close(fd); return -1; }
    return fd;
}

void mpu6050_close(int fd) {
    if (fd >= 0) close(fd);
}

int mpu6050_init(int fd) {
    // Wake up device: set PWR_MGMT_1 = 0
    return i2c_write_reg(fd, REG_PWR_MGMT_1, 0x00);
}

int mpu6050_read_sample(int fd, mpu6050_sample_t *out) {
    if (!out) return -1;

    uint8_t b[14];
    if (i2c_read_regs(fd, REG_ACCEL_XOUT, b, sizeof(b)) != 0) return -1;

    int16_t ax = be16(&b[0]);
    int16_t ay = be16(&b[2]);
    int16_t az = be16(&b[4]);
    int16_t t  = be16(&b[6]);
    int16_t gx = be16(&b[8]);
    int16_t gy = be16(&b[10]);
    int16_t gz = be16(&b[12]);

    // Default full-scale after reset:
    // accel: +/-2g => 16384 LSB/g
    // gyro:  +/-250 dps => 131 LSB/(dps)
    out->ax_g = (double)ax / 16384.0;
    out->ay_g = (double)ay / 16384.0;
    out->az_g = (double)az / 16384.0;

    out->gx_dps = (double)gx / 131.0;
    out->gy_dps = (double)gy / 131.0;
    out->gz_dps = (double)gz / 131.0;

    // Temp in °C = (raw / 340) + 36.53
    out->temp_c = ((double)t / 340.0) + 36.53;

    return 0;
}