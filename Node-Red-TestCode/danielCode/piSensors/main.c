// main.c
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <mosquitto.h>

#include "mcp3008.h"
#include "thermistor.h"
#include "transducer_4_20ma.h"
#include "voltage_divider.h"
#include "mpu6050.h"

// ---- System config ----
#define ADC_VREF            3.3
#define LOOP_HZ             2
#define SAMPLES             16
#define MQTT_HOST           "localhost"
#define MQTT_PORT           1883
#define MQTT_TOPIC          "pi/sensors"
#define MQTT_CLIENT_ID      "pi_sensor_node"

// I2C bus for MPU-6050
static const char *I2C_DEV = "/dev/i2c-1";

typedef struct {
    int fd;
    int ch;
} adc_input_t;

static int adc_read(const adc_input_t *in) {
    if (!in) return -1;
    return mcp3008_read(in->fd, in->ch);
}

int main(void) {

    // ---- Open ADCs ----
    int fd_adc0 = mcp3008_open("/dev/spidev1.0", 0, 1000000);
    int fd_adc1 = mcp3008_open("/dev/spidev1.1", 0, 1000000);
    int fd_adc2 = mcp3008_open("/dev/spidev1.2", 0, 1000000);

    if (fd_adc0 < 0 || fd_adc1 < 0 || fd_adc2 < 0) {
        perror("mcp3008_open");
        if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
        if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
        if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
        return 1;
    }

    // ---- Open IMU ----
    int imu_fd = mpu6050_open(I2C_DEV);
    if (imu_fd < 0) {
        perror("mpu6050_open");
        if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
        if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
        if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
        return 1;
    }

    if (mpu6050_init(imu_fd) != 0) {
        fprintf(stderr, "MPU-6050 init failed\n");
        mpu6050_close(imu_fd);
        if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
        if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
        if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
        return 1;
    }

    // ---- MQTT Init ----
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(MQTT_CLIENT_ID, true, NULL);
    if (!mosq) {
        fprintf(stderr, "mosquitto_new failed\n");
        return 1;
    }

    // Auto-reconnect loop — keeps retrying until broker is up
    while (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT connect failed, retrying in 5s...\n");
        sleep(5);
    }
    printf("MQTT connected to %s:%d\n", MQTT_HOST, MQTT_PORT);

    // Start mosquitto network loop in background thread
    mosquitto_loop_start(mosq);

    // ---- Sensor Parameters ----
    thermistor_cfg_t tcfg = {
        .vref           = ADC_VREF,
        .r_fixed_ohms   = 100000.0,
        .r0_ohms        = 100000.0,
        .t0_c           = 25.0,
        .beta           = 3950.0
    };

    transducer_cfg_t icfg = {
        .vref       = ADC_VREF,
        .shunt_ohms = 165.0,
        .ma_min     = 4.0,
        .ma_max     = 20.0,
        .eng_min    = 0.0,
        .eng_max    = 100.0
    };

    voltage_divider_cfg_t vcfg = {
        .vref       = ADC_VREF,
        .r_top_ohms = 100000.0,
        .r_bot_ohms = 10000.0
    };

    // ---- ADC Assignments ----
    // ADC0 — Thermistors 1-8
    adc_input_t THERM1  = { fd_adc0, 0 };
    adc_input_t THERM2  = { fd_adc0, 1 };
    adc_input_t THERM3  = { fd_adc0, 2 };
    adc_input_t THERM4  = { fd_adc0, 3 };
    adc_input_t THERM5  = { fd_adc0, 4 };
    adc_input_t THERM6  = { fd_adc0, 5 };
    adc_input_t THERM7  = { fd_adc0, 6 };
    adc_input_t THERM8  = { fd_adc0, 7 };
    // ADC1 — Thermistors 9-10, Transducers 1-3
    adc_input_t THERM9  = { fd_adc1, 0 };
    adc_input_t THERM10 = { fd_adc1, 1 };
    adc_input_t TRANS1  = { fd_adc1, 2 };
    adc_input_t TRANS2  = { fd_adc1, 3 };
    adc_input_t TRANS3  = { fd_adc1, 4 };
    // ADC2 — Voltages 1-7 (fixed: was duplicate VOLTS6, added VOLTS7)
    adc_input_t VOLTS1  = { fd_adc2, 0 };
    adc_input_t VOLTS2  = { fd_adc2, 1 };
    adc_input_t VOLTS3  = { fd_adc2, 2 };
    adc_input_t VOLTS4  = { fd_adc2, 3 };
    adc_input_t VOLTS5  = { fd_adc2, 4 };
    adc_input_t VOLTS6  = { fd_adc2, 5 };
    adc_input_t VOLTS7  = { fd_adc2, 6 };  // fixed: was duplicate VOLTS6

    adc_input_t THERMS[] = { THERM1, THERM2, THERM3, THERM4, THERM5,
                              THERM6, THERM7, THERM8, THERM9, THERM10 };
    const int N_THERMS = (int)(sizeof(THERMS) / sizeof(THERMS[0]));

    adc_input_t TRANS[]  = { TRANS1, TRANS2, TRANS3 };
    const int N_TRANS  = (int)(sizeof(TRANS) / sizeof(TRANS[0]));

    adc_input_t VOLTS[]  = { VOLTS1, VOLTS2, VOLTS3, VOLTS4,
                              VOLTS5, VOLTS6, VOLTS7 };
    const int N_VOLTS  = (int)(sizeof(VOLTS) / sizeof(VOLTS[0]));

    // ---- Main Loop ----
    const useconds_t loop_us = (useconds_t)(1000000.0 / LOOP_HZ);
    char json[2048];

    while (1) {
        mpu6050_sample_t s = {0};
        int imu_ok = (mpu6050_read_sample(imu_fd, &s) == 0);

        // Read all thermistors
        double therm_c[10];
        for (int i = 0; i < N_THERMS; i++) {
            int adc = adc_read(&THERMS[i]);
            therm_c[i] = thermistor_c_from_adc(adc, &tcfg);
        }

        // Read all transducers
        double trans_ma[3], trans_eng[3];
        for (int i = 0; i < N_TRANS; i++) {
            int adc = adc_read(&TRANS[i]);
            trans_ma[i]  = transducer_from_adc(adc, &icfg);
            trans_eng[i] = transducer_eng_from_ma(trans_ma[i], &icfg);
        }

        // Read all voltages
        double volts_v[7];
        for (int i = 0; i < N_VOLTS; i++) {
            int adc = adc_read(&VOLTS[i]);
            volts_v[i] = voltage_from_adc(adc, &vcfg);
        }

        // ---- Build JSON payload ----
        int len = snprintf(json, sizeof(json),
            "{"
            "\"therm1\":%.2f,\"therm2\":%.2f,\"therm3\":%.2f,"
            "\"therm4\":%.2f,\"therm5\":%.2f,\"therm6\":%.2f,"
            "\"therm7\":%.2f,\"therm8\":%.2f,\"therm9\":%.2f,"
            "\"therm10\":%.2f,"
            "\"trans1_ma\":%.2f,\"trans1_psi\":%.2f,"
            "\"trans2_ma\":%.2f,\"trans2_psi\":%.2f,"
            "\"trans3_ma\":%.2f,\"trans3_psi\":%.2f,"
            "\"volt1\":%.2f,\"volt2\":%.2f,\"volt3\":%.2f,"
            "\"volt4\":%.2f,\"volt5\":%.2f,\"volt6\":%.2f,"
            "\"volt7\":%.2f,"
            "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
            "\"imu_temp\":%.2f"
            "}",
            therm_c[0], therm_c[1], therm_c[2], therm_c[3], therm_c[4],
            therm_c[5], therm_c[6], therm_c[7], therm_c[8], therm_c[9],
            trans_ma[0], trans_eng[0],
            trans_ma[1], trans_eng[1],
            trans_ma[2], trans_eng[2],
            volts_v[0], volts_v[1], volts_v[2], volts_v[3],
            volts_v[4], volts_v[5], volts_v[6],
            imu_ok ? s.ax_g    : 0.0,
            imu_ok ? s.ay_g    : 0.0,
            imu_ok ? s.az_g    : 0.0,
            imu_ok ? s.gx_dps  : 0.0,
            imu_ok ? s.gy_dps  : 0.0,
            imu_ok ? s.gz_dps  : 0.0,
            imu_ok ? s.temp_c  : 0.0
        );

        // ---- Publish to MQTT ----
        int rc = mosquitto_publish(mosq, NULL, MQTT_TOPIC, len, json, 0, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "MQTT publish failed: %s\n", mosquitto_strerror(rc));
        } else {
            printf("Published: %s\n", json);  // optional, remove in production
        }

        usleep(loop_us);
    }

    // Cleanup
    mosquitto_loop_stop(mosq, true);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    mpu6050_close(imu_fd);
    if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
    if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
    if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
    return 0;
}
