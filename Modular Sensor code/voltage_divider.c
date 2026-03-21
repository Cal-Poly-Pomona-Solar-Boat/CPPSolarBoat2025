// voltage_divider.c
#include "voltage_divider.h"
#include <math.h>

static double adc_to_v(int adc, double vref) {
    return (adc * vref) / 1023.0;
}

double voltage_from_adc(int adc_counts, const voltage_divider_cfg_t *cfg) {
    if (!cfg) return NAN;
    if (adc_counts < 0) return NAN;

    double vnode = adc_to_v(adc_counts, cfg->vref);
    return vnode * (cfg->r_top_ohms + cfg->r_bot_ohms) / cfg->r_bot_ohms;
}