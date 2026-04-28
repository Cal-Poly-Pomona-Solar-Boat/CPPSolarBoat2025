#include <stdio.h>
#include <unistd.h>

#include "mcp3008.h"
#include "transducer_4_20ma.h"

#define ADC_VREF 3.3
#define LOOP_HZ  2

typedef struct {
    int fd;
    int ch;
} adc_input_t;

static int adc_read(const adc_input_t *in) {
    if (!in) return -1;
    return mcp3008_read(in->fd, in->ch);
}

int main(void) {
    int fd_adc1 = mcp3008_open("/dev/spidev1.1", 0, 1000000);
    if (fd_adc1 < 0) {
        perror("mcp3008_open");
        return 1;
    }

    transducer_cfg_t icfg = {
        .vref = ADC_VREF,
        .shunt_ohms = 165.0,
        .ma_min = 4.0,
        .ma_max = 20.0,
        .eng_min = 0.0,
        .eng_max = 100.0
    };

    adc_input_t TRANS1 = { fd_adc1, 2 };

    const useconds_t loop_us = (useconds_t)(1000000.0 / LOOP_HZ);

    while (1) {
        int adc = adc_read(&TRANS1);
        double current = transducer_from_adc(adc, &icfg);
        double eng = transducer_eng_from_ma(current, &icfg);

        printf("TRANS01 | adc=%4d | mA=%6.2f | Eng=%6.2f\n",
               adc, current, eng);

        usleep(loop_us);
    }

    mcp3008_close(fd_adc1);
    return 0;
}
