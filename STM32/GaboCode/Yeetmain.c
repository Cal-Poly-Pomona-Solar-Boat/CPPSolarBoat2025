/* Gabo and Nathan
 * STM32G474RE — AS5600 steering + closed-loop stepper control (bench test)
 *
 * AS5600 #1 : steering wheel command   (I2C3, addr 0x36)
 * AS5600 #2 : rudder left  feedback    (I2C4, addr 0x36)
 * AS5600 #3 : rudder right feedback    (I2C2, addr 0x36)
 *
 * Left  DM860I : TIM8  CH1 (PB6) PUL | PB4 DIR | PB5 ENA
 * Right DM860I : TIM15 CH2 (PB3) PUL | PB10 DIR | PC3 ENA
 *
 * Pulse math:
 *   Target output: 250 RPM → motor 5000 RPM → 83.33 rev/s
 *   Pulse freq = 83.33 × 1600 pulses/rev = 133,333 Hz
 *   TIM clk 170 MHz, prescaler 0 → ARR = 170e6/133333 − 1 = 1274
 *   CMP = ARR/2 = 637 → 50% duty cycle
 *   DM860I max = 200 kHz → within spec
 */

/* ================================================================
 * Includes — main.h MUST be first; it pulls in stm32g4xx_hal.h
 * which defines TIM_HandleTypeDef, GPIO_*, HAL_*, uint8_t, etc.
 * ================================================================ */
#include "main.h"
#include <math.h>
#include <stdio.h>

/* ================================================================
 * Peripheral handles (declared extern in main.h via CubeMX,
 * defined here so the linker finds them)
 * ================================================================ */
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
COM_InitTypeDef   BspCOMInit;

/* ================================================================
 * Forward declarations (static functions defined below main)
 * ================================================================ */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);

/* ================================================================
 * AS5600 raw readings
 * ================================================================ */
static uint8_t steering_data[2];
static float   steering_deg = 0.0f;

static uint8_t fb_left_data[2];
static float   fb_left_deg  = 0.0f;

static uint8_t fb_right_data[2];
static float   fb_right_deg = 0.0f;

/* ================================================================
 * Timing
 * ================================================================ */
#define RATE_STEERING_MS       2U
#define RATE_STEERING_CTRL_MS  2U

static uint32_t last_tick_steering      = 0;
static uint32_t last_tick_steering_ctrl = 0;

/* ================================================================
 * Pulse / ramp config
 *
 * STEP_RATE_MAX is the PID output scale — unitless ceiling.
 * PULSE_FREQ_TARGET is the actual timer frequency at full command.
 * The two are decoupled so PID gains don't need to change if you
 * retune the max speed.
 * ================================================================ */
#define TIM_CLK_HZ               170000000UL

#define STEP_RATE_MAX            8000.0f     // PID output ceiling (unitless)

#define PULSE_FREQ_TARGET        133333.0f   // Hz → 250 RPM output shaft
#define PULSE_FREQ_START           2000.0f   // Hz — jump-start, avoids stall at 0
#define PULSE_FREQ_MIN              500.0f   // Hz — below this, hold stopped
#define PULSE_FREQ_MAX           133333.0f   // Hz — hard cap

/*
 * Trapezoidal ramp rate: Hz added/removed per 2 ms control tick.
 * 133333 / (500 ms / 2 ms) = 533 Hz/tick → full speed in ~0.5 s
 * Increase to ramp faster, decrease to ramp slower.
 */
#define RAMP_ACCEL_HZ_PER_TICK     533.0f

static float freq_out_left  = 0.0f;
static float freq_out_right = 0.0f;

/* ================================================================
 * Steering control limits / PID
 * ================================================================ */
#define RUDDER_DEG_MAX          35.0f
#define RUDDER_DEG_MIN         -35.0f
#define WHEEL_DEG_RANGE         35.0f

#define STEER_DEADBAND_DEG      0.3f
#define STEER_INT_ZONE_DEG      5.0f

#define STEER_KP                200.0f
#define STEER_KI                 40.0f
#define STEER_KD                  8.0f

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float int_min, int_max;
} PID_t;

static PID_t steer_pid = {
    .kp = STEER_KP, .ki = STEER_KI, .kd = STEER_KD,
    .integral = 0.0f, .prev_error = 0.0f,
    .out_min = -STEP_RATE_MAX, .out_max = STEP_RATE_MAX,
    .int_min = -2000.0f,       .int_max =  2000.0f
};

static float steering_cmd_ref_deg = -1.0f;
static float steering_fb_ref_deg  = -1.0f;

static float steering_cmd_deg   = 0.0f;
static float steering_fb_deg    = 0.0f;
static float steering_error_deg = 0.0f;
static float step_rate_cmd      = 0.0f;

/* ================================================================
 * Math helpers
 * ================================================================ */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float angle_wrap_360(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return deg;
}

static float angle_error_deg_fn(float target, float feedback)
{
    float err = target - feedback;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

static float circular_mean_deg(float a, float b)
{
    float x = cosf(a * (float)M_PI / 180.0f) + cosf(b * (float)M_PI / 180.0f);
    float y = sinf(a * (float)M_PI / 180.0f) + sinf(b * (float)M_PI / 180.0f);
    return angle_wrap_360(atan2f(y, x) * 180.0f / (float)M_PI);
}

static float pid_update(PID_t *pid, float error, float dt)
{
    if ((error > 0.0f && pid->prev_error < 0.0f) ||
        (error < 0.0f && pid->prev_error > 0.0f))
        pid->integral = 0.0f;

    if (fabsf(error) < STEER_INT_ZONE_DEG) {
        pid->integral += error * pid->ki * dt;
        pid->integral  = clampf(pid->integral, pid->int_min, pid->int_max);
    }

    float output = pid->kp * error
                 + pid->integral
                 + pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return clampf(output, pid->out_min, pid->out_max);
}

/* ================================================================
 * Low-level timer write — shared by both axes
 *
 * Writes ARR and CCR for the requested frequency, then fires
 * an update event (EGR=UG) so values latch immediately instead
 * of waiting for the next overflow.
 * ================================================================ */
static void _set_timer_freq(TIM_HandleTypeDef *htim, uint32_t channel,
                             float freq_hz)
{
    if (freq_hz < PULSE_FREQ_MIN) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
        return;
    }
    uint32_t arr = (uint32_t)(TIM_CLK_HZ / freq_hz) - 1UL;
    if (arr < 10U)    arr = 10U;
    if (arr > 65535U) arr = 65535U;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2UL);   // 50% duty
    htim->Instance->EGR = TIM_EGR_UG;                  // latch immediately
}

/* ================================================================
 * Left stepper (TIM8 CH1 / PB6 PUL / PB4 DIR / PB5 ENA)
 * ================================================================ */
static void stepper_enable(uint8_t en)
{
    HAL_GPIO_WritePin(GPIOB, EN_L_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void stepper_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    freq_out_left = 0.0f;
    stepper_enable(0);
}

static void steering_apply_output(float signed_rate_cmd)
{
    float target_freq = (fabsf(signed_rate_cmd) / STEP_RATE_MAX) * PULSE_FREQ_TARGET;
    target_freq = clampf(target_freq, 0.0f, PULSE_FREQ_MAX);

    if (target_freq < PULSE_FREQ_MIN) {
        freq_out_left -= RAMP_ACCEL_HZ_PER_TICK;
        if (freq_out_left <= 0.0f) { stepper_stop(); return; }
    } else {
        if (freq_out_left < PULSE_FREQ_START)
            freq_out_left = PULSE_FREQ_START;           // jump to safe start freq
        float delta = clampf(target_freq - freq_out_left,
                             -RAMP_ACCEL_HZ_PER_TICK,
                              RAMP_ACCEL_HZ_PER_TICK);
        freq_out_left = clampf(freq_out_left + delta, PULSE_FREQ_MIN, PULSE_FREQ_MAX);
    }

    HAL_GPIO_WritePin(GPIOB, DIR_L_Pin,
        signed_rate_cmd >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    stepper_enable(1);
    _set_timer_freq(&htim8, TIM_CHANNEL_1, freq_out_left);
}

/* ================================================================
 * Right stepper (TIM15 CH2 / PB3 PUL / PB10 DIR / PC3 ENA)
 * ================================================================ */
static void stepper_right_enable(uint8_t en)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void stepper_right_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    freq_out_right = 0.0f;
    stepper_right_enable(0);
}

static void steering_apply_output_right(float signed_rate_cmd)
{
    float target_freq = (fabsf(signed_rate_cmd) / STEP_RATE_MAX) * PULSE_FREQ_TARGET;
    target_freq = clampf(target_freq, 0.0f, PULSE_FREQ_MAX);

    if (target_freq < PULSE_FREQ_MIN) {
        freq_out_right -= RAMP_ACCEL_HZ_PER_TICK;
        if (freq_out_right <= 0.0f) { stepper_right_stop(); return; }
    } else {
        if (freq_out_right < PULSE_FREQ_START)
            freq_out_right = PULSE_FREQ_START;
        float delta = clampf(target_freq - freq_out_right,
                             -RAMP_ACCEL_HZ_PER_TICK,
                              RAMP_ACCEL_HZ_PER_TICK);
        freq_out_right = clampf(freq_out_right + delta, PULSE_FREQ_MIN, PULSE_FREQ_MAX);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,
        signed_rate_cmd >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    stepper_right_enable(1);
    _set_timer_freq(&htim15, TIM_CHANNEL_2, freq_out_right);
}

/* ================================================================
 * AS5600 read helper
 * ================================================================ */
static float as5600_read_deg(I2C_HandleTypeDef *hi2c, uint8_t *buf)
{
    uint8_t addr = 0x36 << 1;
    if (HAL_I2C_Mem_Read(hi2c, addr, 0x0E, I2C_MEMADD_SIZE_8BIT,
                         &buf[0], 1, HAL_MAX_DELAY) != HAL_OK) return -1.0f;
    if (HAL_I2C_Mem_Read(hi2c, addr, 0x0F, I2C_MEMADD_SIZE_8BIT,
                         &buf[1], 1, HAL_MAX_DELAY) != HAL_OK) return -1.0f;
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    return (raw * 360.0f) / 4096.0f;
}

/* ================================================================
 * main
 * ================================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_I2C4_Init();
    MX_TIM8_Init();
    MX_TIM15_Init();

    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) Error_Handler();

    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim8,  TIM_CHANNEL_1, 0);
    stepper_enable(0);

    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    stepper_right_enable(0);

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ---- Read all three AS5600s @ 500 Hz ---- */
        if (now - last_tick_steering >= RATE_STEERING_MS)
        {
            float v;
            v = as5600_read_deg(&hi2c3, steering_data);
            if (v >= 0.0f) steering_deg = v;

            v = as5600_read_deg(&hi2c4, fb_left_data);
            if (v >= 0.0f) fb_left_deg = v;

            v = as5600_read_deg(&hi2c2, fb_right_data);
            if (v >= 0.0f) fb_right_deg = v;

            last_tick_steering = now;
        }

        /* ---- Closed-loop control @ 500 Hz ---- */
        if (now - last_tick_steering_ctrl >= RATE_STEERING_CTRL_MS)
        {
            const float dt = RATE_STEERING_CTRL_MS / 1000.0f;

            /* Boot-zero capture (runs once) */
            if (steering_cmd_ref_deg < 0.0f)
                steering_cmd_ref_deg = angle_wrap_360(steering_deg);
            if (steering_fb_ref_deg < 0.0f)
                steering_fb_ref_deg = circular_mean_deg(fb_left_deg, fb_right_deg);

            /* Wheel delta → rudder setpoint */
            float wheel_delta = angle_error_deg_fn(
                                    angle_wrap_360(steering_deg),
                                    steering_cmd_ref_deg);
            steering_cmd_deg = clampf(
                (wheel_delta / WHEEL_DEG_RANGE) * RUDDER_DEG_MAX,
                RUDDER_DEG_MIN, RUDDER_DEG_MAX);

            /* Rudder feedback → ±35 deg frame */
            float raw_fb = circular_mean_deg(fb_left_deg, fb_right_deg);
            steering_fb_deg = clampf(
                angle_error_deg_fn(raw_fb, steering_fb_ref_deg),
                RUDDER_DEG_MIN, RUDDER_DEG_MAX);

            /* PID → steppers */
            steering_error_deg = steering_cmd_deg - steering_fb_deg;

            if (fabsf(steering_error_deg) < STEER_DEADBAND_DEG) {
                steer_pid.integral *= 0.90f;
                stepper_stop();
                stepper_right_stop();
            } else {
                step_rate_cmd = pid_update(&steer_pid, steering_error_deg, dt);
                steering_apply_output(step_rate_cmd);
                steering_apply_output_right(step_rate_cmd);
            }

            /* Debug @ 10 Hz */
            static uint32_t last_print = 0;
            if (now - last_print >= 100U) {
                printf("CMD=%.2f  FB=%.2f  ERR=%.2f  Lf=%.0f  Rf=%.0f Hz\r\n",
                       steering_cmd_deg, steering_fb_deg, steering_error_deg,
                       freq_out_left, freq_out_right);
                last_print = now;
            }

            last_tick_steering_ctrl = now;
        }
    }
}

/* ================================================================
 * System Clock — HSE → PLL → 170 MHz
 * ================================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV6;
    RCC_OscInitStruct.PLL.PLLN       = 85;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

/* ================================================================
 * Peripheral init (from CubeMX — do not modify)
 * ================================================================ */
static void MX_I2C2_Init(void)
{
    hi2c2.Instance              = I2C2;
    hi2c2.Init.Timing           = 0x40B285C2;
    hi2c2.Init.OwnAddress1      = 0;
    hi2c2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2      = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) Error_Handler();
}

static void MX_I2C3_Init(void)
{
    hi2c3.Instance              = I2C3;
    hi2c3.Init.Timing           = 0x40B285C2;
    hi2c3.Init.OwnAddress1      = 0;
    hi2c3.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2      = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) Error_Handler();
}

static void MX_I2C4_Init(void)
{
    hi2c4.Instance              = I2C4;
    hi2c4.Init.Timing           = 0x40B285C2;
    hi2c4.Init.OwnAddress1      = 0;
    hi2c4.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2      = 0;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) Error_Handler();
}

static void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig           = {0};
    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 0;
    htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim8.Init.Period            = 65535;
    htim8.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim8);
}

static void MX_TIM15_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig           = {0};
    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim15.Instance               = TIM15;
    htim15.Init.Prescaler         = 0;
    htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim15.Init.Period            = 65535;
    htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim15);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DIR_R_Pin | DIR_L_Pin | EN_L_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = EN_R_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_R_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = DIR_R_Pin | DIR_L_Pin | EN_L_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ================================================================
 * Error handler
 * ================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
