// mcp3008.h
#pragma once

int mcp3008_open(const char *spidev, unsigned mode, unsigned speed_hz);
void mcp3008_close(int fd);
int mcp3008_read(int fd, int channel); // returns 0..1023 or -1