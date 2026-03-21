// mcp3008.c
#define _POSIX_C_SOURCE 200809L
#include "mcp3008.h"

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int mcp3008_open(const char *spidev, unsigned mode, unsigned speed_hz) {
    int fd = open(spidev, O_RDWR);
    if (fd < 0) return -1;

    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) { close(fd); return -1; }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { close(fd); return -1; }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) { close(fd); return -1; }

    return fd;
}

void mcp3008_close(int fd) {
    if (fd >= 0) close(fd);
}

int mcp3008_read(int fd, int channel) {
    if (channel < 0 || channel > 7) return -1;

    // MCP3008 protocol:
    // tx[0] = 1 (start)
    // tx[1] = (8 + channel) << 4 (single-ended)
    // tx[2] = 0
    uint8_t tx[3] = { 0x01, (uint8_t)((0x08 | channel) << 4), 0x00 };
    uint8_t rx[3] = { 0, 0, 0 };

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .delay_usecs = 0,
        .speed_hz = 0,         // use device setting
        .bits_per_word = 0
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) return -1;

    int value = ((rx[1] & 0x03) << 8) | rx[2];
    return value;
}