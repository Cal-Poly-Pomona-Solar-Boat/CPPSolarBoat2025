/* * Gabo and Nathan - FULL SYSTEM CODE
 * Target: STM32G474RE
 * Hardware: AS5600 (x3) via I2C, DM860I Driver (x2) via PWM
 * Motor Target: 1000 RPM (26,667 Hz @ 1600 Microsteps)
 */

#include "main.h"
#include <math.h>
#include <stdio.h>

/* ================================================================
 * Peripheral handles
 * ================================================================ */
I2C_HandleTypeDef hi2c2, hi2c3, hi2c4;
TIM_HandleTypeDef htim2, htim8, htim15;
UART_HandleTypeDef hlpuart1; // Assuming LPUART1 for printf
COM_InitTypeDef   BspCOMInit;

/* ================================================================
 * Constants & PID Configuration
 * ================================================================ */
#define TIM_CLK_HZ               170000000UL
#define STEP_RATE_MAX            8000.0f    
#define PULSE_FREQ_TARGET        26667.0f   // 1000 RPM limit
#define PULSE_FREQ_START         1000.0f    
#define PULSE_FREQ_MIN           300.0f     
#define PULSE_FREQ_MAX           26667.0f   
#define RAMP_ACCEL_HZ_PER_TICK   350.0f     

#define RUDDER_DEG_MAX           35.0f
#define RUDDER_DEG_MIN          -35.0f
#define WHEEL_DEG_RANGE          35.0f
#define STEER_DEADBAND_DEG       0.4f       
#define STEER_INT_ZONE_DEG       5.0f

#define STEER_KP                 120.0f     
#define STEER_KI                 30.0f
#define STEER_KD                 10.0f

typedef struct {
    float kp, ki, kd, integral, prev_error;
    float out_min, out_max, int_min, int_max;
} PID_t;

static PID_t steer_pid = {
    .kp = STEER_KP, .ki = STEER_KI, .kd = STEER_KD,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -STEP_RATE_MAX, .out_max = STEP_RATE_MAX,
    .int_min = -1500.0f, .int_max = 1500.0f
};

/* ================================================================
 * Global Variables
 * ================================================================ */
static float steering_deg = 0.0f, fb_left_deg = 0.0f, fb_right_deg = 0.0f;
static uint32_t last_tick_steering = 0, last_tick_steering_ctrl = 0;
static float steering_cmd_ref_deg = -1.0f, steering_fb_ref_deg = -1.0f;
static float freq_out_left = 0.0f;

/* ================================================================
 * Forward Declarations
 * ================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
void Error_Handler(void);

/* ================================================================
 * Math & Logic Helpers
 * ================================================================ */
static float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static float angle_wrap_360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

static float angle_error_deg_fn(float target, float feedback) {
    float err = target - feedback;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

static float circular_mean_deg(float a, float b) {
    float x = cosf(a * M_PI / 180.0f) + cosf(b * M_PI / 180.0f);
    float y = sinf(a * M_PI / 180.0f) + sinf(b * M_PI / 180.0f);
    return angle_wrap_360(atan2f(y, x) * 180.0f / M_PI);
}

static float pid_update(PID_t *pid, float error, float dt) {
    if ((error > 0.0f && pid->prev_error < 0.0f) || (error < 0.0f && pid->prev_error > 0.0f))
        pid->integral = 0.0f;
    if (fabsf(error) < STEER_INT_ZONE_DEG) {
        pid->integral = clampf(pid->integral + (error * pid->ki * dt), pid->int_min, pid->int_max);
    }
    float output = (pid->kp * error) + pid->integral + (pid->kd * (error - pid->prev_error) / dt);
    pid->prev_error = error;
    return clampf(output, pid->out_min, pid->out_max);
}

/* ================================================================
 * Stepper Movement Engine
 * ================================================================ */
static void _set_timer_freq(TIM_HandleTypeDef *htim, uint32_t channel, float freq_hz) {
    if (freq_hz < PULSE_FREQ_MIN) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
        return;
    }
    uint32_t arr = (uint32_t)(TIM_CLK_HZ / freq_hz) - 1UL;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2UL);
    htim->Instance->EGR = TIM_EGR_UG;
}

static void stepper_stop(void) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    freq_out_left = 0.0f;
    HAL_GPIO_WritePin(GPIOB, EN_L_Pin, GPIO_PIN_SET); // Disabled (Active High)
    HAL_GPIO_WritePin(GPIOC, EN_R_Pin, GPIO_PIN_SET); // Disabled
}

static void steering_apply_motion(float signed_rate_cmd) {
    float target_freq = (fabsf(signed_rate_cmd) / STEP_RATE_MAX) * PULSE_FREQ_TARGET;
    target_freq = clampf(target_freq, 0.0f, PULSE_FREQ_MAX);

    // Direction stabilization hysteresis
    if (fabsf(signed_rate_cmd) > 0.1f) {
        GPIO_PinState dir_state = (signed_rate_cmd > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(GPIOB, DIR_L_Pin, dir_state);
        HAL_GPIO_WritePin(GPIOB, DIR_R_Pin, dir_state);
    }

    if (target_freq < PULSE_FREQ_MIN) {
        freq_out_left -= RAMP_ACCEL_HZ_PER_TICK;
        if (freq_out_left < 0) { stepper_stop(); return; }
    } else {
        if (freq_out_left < PULSE_FREQ_START) freq_out_left = PULSE_FREQ_START;
        float delta = clampf(target_freq - freq_out_left, -RAMP_ACCEL_HZ_PER_TICK, RAMP_ACCEL_HZ_PER_TICK);
        freq_out_left = clampf(freq_out_left + delta, PULSE_FREQ_MIN, PULSE_FREQ_MAX);
    }

    HAL_GPIO_WritePin(GPIOB, EN_L_Pin, GPIO_PIN_RESET); // Enabled
    HAL_GPIO_WritePin(GPIOC, EN_R_Pin, GPIO_PIN_RESET); // Enabled
    _set_timer_freq(&htim8, TIM_CHANNEL_1, freq_out_left);
    _set_timer_freq(&htim15, TIM_CHANNEL_2, freq_out_left);
}

static float as5600_read_deg(I2C_HandleTypeDef *hi2c) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c, 0x36 << 1, 0x0E, 1, &buf[0], 1, 10) != HAL_OK) return -1.0f;
    if (HAL_I2C_Mem_Read(hi2c, 0x36 << 1, 0x0F, 1, &buf[1], 1, 10) != HAL_OK) return -1.0f;
    return (((uint16_t)buf[0] << 8) | buf[1]) * 360.0f / 4096.0f;
}

/* ================================================================
 * Main Process
 * ================================================================ */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C2_Init(); MX_I2C3_Init(); MX_I2C4_Init();
    MX_TIM2_Init(); MX_TIM8_Init(); MX_TIM15_Init();

    BspCOMInit.BaudRate = 115200;
    BSP_COM_Init(COM1, &BspCOMInit);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    stepper_stop();

    while (1) {
        uint32_t now = HAL_GetTick();

        // 500Hz Sensor Polling
        if (now - last_tick_steering >= 2) {
            float v;
            if ((v = as5600_read_deg(&hi2c3)) >= 0) steering_deg = v;
            if ((v = as5600_read_deg(&hi2c4)) >= 0) fb_left_deg  = v;
            if ((v = as5600_read_deg(&hi2c2)) >= 0) fb_right_deg = v;
            last_tick_steering = now;
        }

        // 500Hz PID & Control logic
        if (now - last_tick_steering_ctrl >= 2) {
            if (steering_cmd_ref_deg < 0) steering_cmd_ref_deg = angle_wrap_360(steering_deg);
            if (steering_fb_ref_deg < 0)  steering_fb_ref_deg  = circular_mean_deg(fb_left_deg, fb_right_deg);

            float wheel_delta = angle_error_deg_fn(steering_deg, steering_cmd_ref_deg);
            float target_rudder = clampf((wheel_delta / WHEEL_DEG_RANGE) * RUDDER_DEG_MAX, RUDDER_DEG_MIN, RUDDER_DEG_MAX);
            
            float raw_fb = circular_mean_deg(fb_left_deg, fb_right_deg);
            float steering_fb_deg = angle_error_deg_fn(raw_fb, steering_fb_ref_deg);
            float error = target_rudder - steering_fb_deg;

            if (fabsf(error) < STEER_DEADBAND_DEG) {
                stepper_stop();
                steer_pid.integral *= 0.8f;
                steer_pid.prev_error = 0;
            } else {
                float cmd = pid_update(&steer_pid, error, 0.002f);
                steering_apply_motion(cmd);
            }
            last_tick_steering_ctrl = now;
        }
    }
}

/* ================================================================
 * Clock & Peripheral Initialization (Standard CubeMX Style)
 * ================================================================ */

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_I2C2_Init(void) {
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x40B285C2;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c2);
}

static void MX_I2C3_Init(void) {
    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = 0x40B285C2;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c3);
}

static void MX_I2C4_Init(void) {
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = 0x40B285C2;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c4);
}

static void MX_TIM2_Init(void) {
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 169;
    htim2.Init.Period = 4294967295;
    HAL_TIM_Base_Init(&htim2);
}

static void MX_TIM8_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.Period = 65535;
    HAL_TIM_PWM_Init(&htim8);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
}

static void MX_TIM15_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 0;
    htim15.Init.Period = 65535;
    HAL_TIM_PWM_Init(&htim15);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOB, DIR_R_Pin|DIR_L_Pin|EN_L_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, EN_R_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = DIR_R_Pin|DIR_L_Pin|EN_L_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EN_R_Pin;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
