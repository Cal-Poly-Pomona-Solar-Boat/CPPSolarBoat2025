// voltage_divider.h
#pragma once
typedef struct {
    double vref;
    double r_top_ohms;
    double r_bot_ohms;
} voltage_divider_cfg_t;

double voltage_from_adc(int adc_counts, const voltage_divider_cfg_t *cfg);