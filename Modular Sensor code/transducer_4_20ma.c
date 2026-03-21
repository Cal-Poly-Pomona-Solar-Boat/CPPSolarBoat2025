// transducer_4_20ma.c
#include "transducer_4_20ma.h"
#include <math.h>

static double adc_to_v(int adc, double vref) {
    return (adc * vref) / 1023.0;
}

double transducer_from_adc(int adc_counts, const transducer_cfg_t *cfg) {
    if (!cfg) return NAN;
    if (adc_counts < 0) return NAN;

    double v = adc_to_v(adc_counts, cfg->vref);
    double i_a = v / cfg->shunt_ohms;
    return i_a * 1000.0;
}

double transducer_eng_from_ma(double ma, const transducer_cfg_t *cfg) {
    if (!cfg) return NAN;
    if (!isfinite(ma)) return NAN;

    // Map [ma_min, ma_max] -> [eng_min, eng_max]
    double x = (ma - cfg->ma_min) / (cfg->ma_max - cfg->ma_min);
    return cfg->eng_min + x * (cfg->eng_max - cfg->eng_min);
}