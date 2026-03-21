// main.c
//#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>


#include "mcp3008.h"
#include "thermistor.h"
#include "transducer_4_20ma.h"
#include "voltage_divider.h"
#include "mpu6050.h"

// ---- System config ----
#define ADC_VREF            3.3
#define LOOP_HZ             2             // print rate
#define SAMPLES             16              // simple averaging per channel

// SPI1 MCP3008 device 
//static const char *SPI_DEV = "/dev/spidev1.0";

//code for assigning 3 adc chips
typedef struct {
    int fd;     // which MCP3008 (spidev fd)
    int ch;     // which channel 0..7
} adc_input_t;

static int adc_read(const adc_input_t *in) {
    if (!in) return -1;
    return mcp3008_read(in->fd, in->ch);
}
// I2C bus for MPU-6050
static const char *I2C_DEV = "/dev/i2c-1";

static int read_avg_adc(int spi_fd, int ch) {
    long sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int v = mcp3008_read(spi_fd, ch);
        if (v < 0) return -1;
        sum += v;
    }
    return (int)lround((double)sum / (double)SAMPLES);
}

int main(void) {
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

    int imu_fd = mpu6050_open(I2C_DEV);
    if (imu_fd < 0) {
        perror("mpu6050_open");
        //mcp3008_close(spi_fd);
        if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
        if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
        if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
        return 1;
    }

    // Initialize MPU-6050 (wake up, basic config)
    if (mpu6050_init(imu_fd) != 0) {
        fprintf(stderr, "MPU-6050 init failed\n");
        mpu6050_close(imu_fd);
        //mcp3008_close(spi_fd);
        if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
        if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
        if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
        return 1;
    }

    // ---- Sensor paramameterss ----
    //Thermister paramaters
    thermistor_cfg_t tcfg = {
        .vref = ADC_VREF,
        .r_fixed_ohms = 100000.0,      
        .r0_ohms = 100000.0,           
        .t0_c = 25.0,
        .beta = 3950.0                
    };

    // Transducer Parameters
    transducer_cfg_t icfg = {
        .vref = ADC_VREF,
        .shunt_ohms = 165.0,          // example: 165Ω => 20mA -> 3.3V
        .ma_min = 4.0,
        .ma_max = 20.0,
        .eng_min = 0.0,               // engineering units (e.g., PSI) min
        .eng_max = 100.0              // engineering units max
    };

    // Voltage divider scaling
    voltage_divider_cfg_t vcfg = {
        .vref = ADC_VREF,
        .r_top_ohms = 100000.0,       
        .r_bot_ohms = 10000.0         
    };
     //ADC Assignments
     // ADC0 assignments
    adc_input_t THERM1  = { fd_adc0, 0 };
    adc_input_t THERM2  = { fd_adc0, 1 };
    adc_input_t THERM3  = { fd_adc0, 2 };
    adc_input_t THERM4  = { fd_adc0, 3 }; //THERMISTOR 1-6 ARE FOR BATTERIES
    adc_input_t THERM5  = { fd_adc0, 4 };
    adc_input_t THERM6  = { fd_adc0, 5 };
    adc_input_t THERM7  = { fd_adc0, 6 };
    adc_input_t THERM8  = { fd_adc0, 7 };
    //ADC1 assignments
    adc_input_t THERM9 =  { fd_adc1, 0 }; //Two thermistors on second adc
    adc_input_t THERM10 = { fd_adc1, 1 };
    adc_input_t TRANS1 =  { fd_adc1, 2 };//Three transducers
    adc_input_t TRANS2 =  { fd_adc1, 3 };
    adc_input_t TRANS3 =  { fd_adc1, 4 };
    //ADC2 assignments
    adc_input_t VOLTS1  = { fd_adc2, 0 };
    adc_input_t VOLTS2  = { fd_adc2, 1 };
    adc_input_t VOLTS3  = { fd_adc2, 2 };
    adc_input_t VOLTS4  = { fd_adc2, 3 };
    adc_input_t VOLTS5  = { fd_adc2, 4 };
    adc_input_t VOLTS6  = { fd_adc2, 5 };
    adc_input_t VOLTS6  = { fd_adc2, 6 };

    //Store Thermistor values in array
    adc_input_t THERMS[] = {
    THERM1, THERM2, THERM3, THERM4, THERM5,
    THERM6, THERM7, THERM8, THERM9, THERM10
    };
    const int N_THERMS = (int)(sizeof(THERMS) / sizeof(THERMS[0])); 
     adc_input_t TRANS[] = {
    TRANS1, TRANS2, TRANS3
    };
    const int N_TRANS = (int)(sizeof(TRANS) / sizeof(TRANS[0])); 
    adc_input_t VOLTS[] = {
    VOLTS1, VOLTS2, VOLTS3, VOLTS4, VOLTS5,
    VOLTS6
    };
    const int N_VOLTS = (int)(sizeof(VOLTS) / sizeof(VOLTS[0])); 
    // ---- Main loop ----
    const useconds_t loop_us = (useconds_t)(1000000.0 / LOOP_HZ);

    while (1) {
        

        mpu6050_sample_t s = {0};
        int imu_ok = (mpu6050_read_sample(imu_fd, &s) == 0);

        printf("---- Sensors ----\n");

        //Thermistor Print loop
        printf("---- THERMISTORS ----\n");
        for (int i = 0; i < N_THERMS; i++) {
        int adc = adc_read(&THERMS[i]);
        double temp = thermistor_c_from_adc(adc, &tcfg);   
        printf("THERM%02d: adc=%4d  T=%.2f C\n", i + 1, adc, temp);
        }
        printf("\n");
        printf("---- TRANSDUCERS ----\n");
        for (int i = 0; i < N_TRANS; i++) {
        int adc = adc_read(&TRANS[i]);
        double current = transducer_from_adc(adc, &icfg);
        double eng = transducer_eng_from_ma(current, &icfg);
        printf("TRANS%02d | adc=%4d | mA=%6.2f | Eng=%6.2f\n", i + 1, adc, current, eng);
        }
        printf("---- VOLTAGES ----\n");
        for (int i = 0; i < N_VOLTS; i++) {
        int adc = adc_read(&VOLTS[i]);
        double volts = voltage_from_adc(adc, &vcfg);
        printf("VOLT%02d  | adc=%4d | V=%6.2f V\n", i + 1, adc, volts);
        }
       
        if (imu_ok) {
            printf("IMU accel(g):   ax=%.3f ay=%.3f az=%.3f\n", s.ax_g, s.ay_g, s.az_g);
            printf("IMU gyro(dps):  gx=%.2f gy=%.2f gz=%.2f\n", s.gx_dps, s.gy_dps, s.gz_dps);
            printf("IMU temp(C):    %.2f\n", s.temp_c);
        } else {
            printf("IMU: read failed\n");
        }
        printf("\n");

        usleep(loop_us);
    }

    // unreachable, but tidy
    mpu6050_close(imu_fd);
    if (fd_adc0 >= 0) mcp3008_close(fd_adc0);
    if (fd_adc1 >= 0) mcp3008_close(fd_adc1);
    if (fd_adc2 >= 0) mcp3008_close(fd_adc2);
    return 0;
}