// transducer_4_20ma.h
#pragma once
typedef struct {
    double vref;
    double shunt_ohms;  // resistor converting current to voltage
    double ma_min;      // 4.0
    double ma_max;      // 20.0
    double eng_min;     // e.g., 0 PSI
    double eng_max;     // e.g., 100 PSI
} transducer_cfg_t;

double transducer_from_adc(int adc_counts, const transducer_cfg_t *cfg);
double transducer_eng_from_ma(double ma, const transducer_cfg_t *cfg);