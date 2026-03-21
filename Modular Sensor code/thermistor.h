// thermistor.h
#pragma once
typedef struct {
    double vref;
    double r_fixed_ohms;   // top resistor
    double r0_ohms;        // thermistor resistance at t0
    double t0_c;           // typically 25C
    double beta;           // beta parameter
} thermistor_cfg_t;

double thermistor_c_from_adc(int adc_counts, const thermistor_cfg_t *cfg);