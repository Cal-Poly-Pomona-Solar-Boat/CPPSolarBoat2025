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
#define MQTT_HOST           "localhost"
#define MQTT_PORT           1883
#define MQTT_TOPIC          "pi/sensors"
#define MQTT_CLIENT_ID      "pi_sensor_node"

static const char *I2C_DEV = "/dev/i2c-1";

typedef struct {
    int fd;
    int ch;
} adc_input_t;

static int adc_read(const adc_input_t *in) {
    if (!in || in->fd < 0) return 0;
    return mcp3008_read(in->fd, in->ch);
}

int main(void) {
    // ---- Open Hardware ----
    int fd_adc0 = mcp3008_open("/dev/spidev1.0", 0, 1000000);
    int fd_adc1 = mcp3008_open("/dev/spidev1.1", 0, 1000000);
    int fd_adc2 = mcp3008_open("/dev/spidev1.2", 0, 1000000);

    if (fd_adc0 < 0 || fd_adc1 < 0 || fd_adc2 < 0) {
        perror("mcp3008_open failed");
        return 1; 
    }

    int imu_fd = mpu6050_open(I2C_DEV);
    int imu_active = (imu_fd >= 0 && mpu6050_init(imu_fd) == 0);

    // ---- MQTT ----
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new(MQTT_CLIENT_ID, true, NULL);
    if (!mosq) return 1;
    while (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT connection failed, retrying...\n");
        sleep(5);
    }
    mosquitto_loop_start(mosq);

    // ---- Configs ----
    thermistor_cfg_t tcfg = { .vref = ADC_VREF, .r_fixed_ohms = 100000.0, .r0_ohms = 100000.0, .t0_c = 25.0, .beta = 3950.0 };
    transducer_cfg_t icfg = { .vref = ADC_VREF, .shunt_ohms = 165.0, .ma_min = 4.0, .ma_max = 20.0, .eng_min = 0.0, .eng_max = 100.0 };
    voltage_divider_cfg_t vcfg = { .vref = ADC_VREF, .r_top_ohms = 100000.0, .r_bot_ohms = 10000.0 };

    adc_input_t THERMS[] = { {fd_adc0,0}, {fd_adc0,1}, {fd_adc0,2}, {fd_adc0,3}, {fd_adc0,4}, {fd_adc0,5}, {fd_adc0,6}, {fd_adc0,7}, {fd_adc1,0}, {fd_adc1,1} };
    adc_input_t TRANS[]  = { {fd_adc1,2}, {fd_adc1,3}, {fd_adc1,4} };
    adc_input_t VOLTS[]  = { {fd_adc2,0}, {fd_adc2,1}, {fd_adc2,2}, {fd_adc2,3}, {fd_adc2,4}, {fd_adc2,5}, {fd_adc2,6} };

    char json[2048];

    while (1) {
        mpu6050_sample_t s = {0};
        int imu_ok = (imu_active && mpu6050_read_sample(imu_fd, &s) == 0);

        double tc[10], t_ma[3], t_psi[3], v[7];

        // --- Thermistors with Hard Check ---
        for (int i = 0; i < 10; i++) {
            tc[i] = thermistor_c_from_adc(adc_read(&THERMS[i]), &tcfg);
            // Final JSON safety check
            if (!isfinite(tc[i])) tc[i] = 0.0;
        }

        for (int i = 0; i < 3; i++) {
            t_ma[i] = transducer_from_adc(adc_read(&TRANS[i]), &icfg);
            t_psi[i] = transducer_eng_from_ma(t_ma[i], &icfg);
            if (!isfinite(t_psi[i])) t_psi[i] = 0.0;
        }

        for (int i = 0; i < 7; i++) {
            v[i] = voltage_from_adc(adc_read(&VOLTS[i]), &vcfg);
            if (!isfinite(v[i])) v[i] = 0.0;
        }

        snprintf(json, sizeof(json),
            "{"
            "\"therm1\":%.2f,\"therm2\":%.2f,\"therm3\":%.2f,\"therm4\":%.2f,\"therm5\":%.2f,"
            "\"therm6\":%.2f,\"therm7\":%.2f,\"therm8\":%.2f,\"therm9\":%.2f,\"therm10\":%.2f,"
            "\"trans1_ma\":%.2f,\"trans1_psi\":%.2f,"
            "\"trans2_ma\":%.2f,\"trans2_psi\":%.2f,"
            "\"trans3_ma\":%.2f,\"trans3_psi\":%.2f,"
            "\"volt1\":%.2f,\"volt2\":%.2f,\"volt3\":%.2f,\"volt4\":%.2f,\"volt5\":%.2f,\"volt6\":%.2f,\"volt7\":%.2f,"
            "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
            "\"imu_temp\":%.2f"
            "}",
            tc[0], tc[1], tc[2], tc[3], tc[4], tc[5], tc[6], tc[7], tc[8], tc[9],
            t_ma[0], t_psi[0], t_ma[1], t_psi[1], t_ma[2], t_psi[2],
            v[0], v[1], v[2], v[3], v[4], v[5], v[6],
            imu_ok ? s.ax_g : 0.0, imu_ok ? s.ay_g : 0.0, imu_ok ? s.az_g : 0.0,
            imu_ok ? s.gx_dps : 0.0, imu_ok ? s.gy_dps : 0.0, imu_ok ? s.gz_dps : 0.0,
            imu_ok ? s.temp_c : 0.0
        );

        mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(json), json, 0, false);
        usleep(1000000 / LOOP_HZ);
    }
    return 0;
}
