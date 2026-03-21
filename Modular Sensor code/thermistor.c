// thermistor.c
#include "thermistor.h"
#include <math.h>
#include <stdio.h>

static double adc_to_v(int adc, double vref) {
    return (adc * vref) / 1023.0;
}

static double r_therm_from_v(double v, const thermistor_cfg_t *cfg) {
    if (v <= 0.0) return INFINITY;
    if (v >= cfg->vref) return 0.0;
    return cfg->r_fixed_ohms * (v / (cfg->vref - v));
}
double thermistor_c_from_adc(int adc_counts, const thermistor_cfg_t *cfg) { if (!cfg) return NAN; if (adc_counts < 0) return NAN; double v = adc_to_v(adc_counts, cfg->vref); double r = r_therm_from_v(v, cfg); if (!isfinite(r) || r <= 0.0) return NAN;
// ---- DEBUG PRINT ----
    printf("THERM DEBUG | adc=%d  v=%.3f V  r=%.0f ohms\n",
           adc_counts, v, r);

    // Beta equation:
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    double t0_k = cfg->t0_c + 273.15;
    double invT = (1.0 / t0_k) + (1.0 / cfg->beta) * log(r / cfg->r0_ohms);
    double t_k = 1.0 / invT;

    double t_c = t_k - 273.15;

    printf("THERM DEBUG | temp=%.2f C\n", t_c);
    return t_c;
}