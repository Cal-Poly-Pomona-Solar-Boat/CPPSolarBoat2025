#ifndef THERMISTOR_H
#define THERMISTOR_H

/**
 * Configuration for a NTC thermistor in a voltage divider circuit.
 * Circuit: VREF --- [R_FIXED] ---+--- [THERMISTOR] --- GND
 * |
 * ADC PIN
 */
typedef struct {
    double vref;           // ADC reference voltage (e.g., 3.3)
    double r_fixed_ohms;   // Fixed pull-up resistor (e.g., 100000.0)
    double r0_ohms;        // Thermistor resistance at t0 (e.g., 100000.0)
    double t0_c;           // Reference temperature in Celsius (e.g., 25.0)
    double beta;           // Beta parameter (e.g., 3950.0)
} thermistor_cfg_t;

/**
 * Converts raw ADC counts to temperature in Celsius.
 * Returns 0.0 if the sensor is disconnected or math is non-finite.
 */
double thermistor_c_from_adc(int adc_counts, const thermistor_cfg_t *cfg);

#endif
