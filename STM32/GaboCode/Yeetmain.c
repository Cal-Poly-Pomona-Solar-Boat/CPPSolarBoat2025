/* USER CODE BEGIN Header */
/* Gabo and Nathan
/**
******************************************************************************
* @file           : main.c
* @brief          : Sensor Hub + Closed-Loop Steering + FDCAN Transmit
*                   STM32G474RE
*
* Sensors:
*   - AS5600 #1 : magnetic steering angle (I2C3, addr 0x36)  ← steering wheel command
*   - AS5600 #2 : magnetic steering angle (I2C4, addr 0x36)  ← rudder left  feedback
*   - AS5600 #3 : magnetic steering angle (I2C2, addr 0x36)  ← rudder right feedback
*   - ADXL345 #1 : motor 1 vibration      (I2C3, addr 0x53 / 0xA6)
*   - ADXL345 #2 : motor 2 vibration      (I2C3, addr 0x1D / 0x3A)
*   - Hall sensor #1 : motor RPM          (PA0, EXTI0)
*   - Hall sensor #2 : motor RPM          (PA1, EXTI1)
*   - Throttle : 0-5V mapping             (PC0, ADC1)
*
* Stepper outputs:
*   Left  DM860I : TIM8  CH1 (PB6)  PUL | PB4 DIR | PB5 ENA
*   Right DM860I : TIM15 CH2 (PB3)  PUL | PB10 DIR | PC3 ENA
*
* Motor-control notes:
*   - Direct step-rate command in pulses/sec
*   - Signed slew limiting for smooth reversals
*   - 50% duty pulse output
*   - Immediate timer update using EGR=UG
*   - TIM8/TIM15 prescaler set to 169 -> 1 MHz timer clock
*   - AS5600 read uses one 2-byte transaction and masks to 12 bits
*
* Gearbox / pulse math:
*   - Motor microstep setting assumed: 1600 pulses/rev
*   - Gearbox ratio: 20:1
*   - Output deg/pulse = 360 / (1600 * 20) = 0.01125 deg
*   - STEP_RATE_MAX = 8000 pulse/s
*   - Motor speed max = 8000 / 1600 * 60 = 300 RPM
*   - Output speed max = 300 / 20 = 15 RPM
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "can_frames.h"
#include "math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac1;
FDCAN_HandleTypeDef hfdcan1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */

/* --- AS5600 sensors --- */
static uint8_t steering_data[2];
static float   steering_deg = 0.0f;

static uint8_t fb_left_data[2];
static float   fb_left_deg  = 0.0f;

static uint8_t fb_right_data[2];
static float   fb_right_deg = 0.0f;

/* --- Throttle 0-5V (ADC1) --- */
uint16_t throttle_raw = 0;
float    throttle_out = 0.0f;

/* --- ADXL345 #1 motor_VibrationLeft --- */
#define ADXL_SAMPLES  10
#define ADXL_THRESH_X 0.08f
#define ADXL_THRESH_Y 0.08f
#define ADXL_THRESH_Z 0.08f
uint8_t  adxl1_id;
uint8_t  adxl1_data[6];
int16_t  ax1, ay1, az1;
float    accel_x1[10], accel_y1[10], accel_z1[10];
float    mean_x1 = 0.0f, mean_y1 = 0.0f, mean_z1 = 0.0f;
float    sum_x1  = 0.0f, sum_y1  = 0.0f, sum_z1  = 0.0f;
uint8_t  adxl_power_ctl = 0x08;
float    adxl_cal_val   = 0.0039f;
int      adxl1_index = 0;

/* --- ADXL345 #2 motor_VibrationRight --- */
uint8_t  adxl2_id;
uint8_t  adxl2_data[6];
int16_t  ax2, ay2, az2;
float    accel_x2[10], accel_y2[10], accel_z2[10];
float    mean_x2 = 0.0f, mean_y2 = 0.0f, mean_z2 = 0.0f;
float    sum_x2  = 0.0f, sum_y2  = 0.0f, sum_z2  = 0.0f;
int      adxl2_index = 0;

/* --- Hall-effect #1 motor_RpmLeft --- */
#define AVG_SAMPLES           5
#define PULSES_PER_REVOLUTION 4
#define RPM_TIMEOUT           3000000U
#define TICKS_PER_SECOND      1000714.0f
volatile uint32_t current_time               = 0;
volatile uint32_t pulse_1_count              = 0;
volatile uint32_t last_time_1                = 0;
volatile float    rpm_1                      = 0.0f;
volatile float    rpm_1_avg                  = 0.0f;
volatile uint8_t  new_rpm_1_ready            = 0;
volatile float    rpm_1_buffer[AVG_SAMPLES]  = {0};
volatile uint8_t  rpm_1_buffer_index         = 0;

/* --- Hall-effect #2 motor_RpmRight --- */
volatile uint32_t pulse_2_count              = 0;
volatile uint32_t last_time_2                = 0;
volatile float    rpm_2                      = 0.0f;
volatile float    rpm_2_avg                  = 0.0f;
volatile uint8_t  new_rpm_2_ready            = 0;
volatile float    rpm_2_buffer[AVG_SAMPLES]  = {0};
volatile uint8_t  rpm_2_buffer_index         = 0;

/* --- FDCAN filter --- */
FDCAN_FilterTypeDef sFilterConfig;

/* --- Send / service intervals (ms) --- */
#define RATE_STEERING_MS              2U
#define RATE_STEERING_CTRL_MS         2U
#define RATE_THROTTLE_MS             20U
#define RATE_MOTOR_VIBRATION_MS      10U
#define RATE_MOTOR_RPM_MS             1U
#define ADXL_SAMPLE_INTERVAL_MS      (RATE_MOTOR_VIBRATION_MS / ADXL_SAMPLES)

/* --- Tick timestamps --- */
static uint32_t last_tick_steering            = 0;
static uint32_t last_tick_steering_ctrl       = 0;
static uint32_t last_tick_throttle            = 0;
static uint32_t last_tick_motor_vibrationLeft = 0;
static uint32_t last_tick_motor_vibrationRight= 0;
static uint32_t last_tick_motor_rpmLeft       = 0;
static uint32_t last_tick_motor_rpmRight      = 0;

/* ================================================================
 * CLOSED-LOOP STEERING CONTROL
 * ================================================================ */

/* TIM8/TIM15 effective timer clock after PSC=169 */
#define STEP_TIM_PSC                 169U
#define STEP_TIM_CLK_HZ              (170000000.0f / (STEP_TIM_PSC + 1.0f))   /* 1 MHz */

/* Direct step-rate command in pulses/second */
#define STEP_RATE_MAX                8000.0f
#define STEP_RATE_MIN                80.0f
#define STEP_RATE_SLEW_PER_TICK      4000.0f

/* Steering geometry */
#define RUDDER_DEG_MAX               35.0f
#define RUDDER_DEG_MIN              -35.0f
#define WHEEL_DEG_RANGE              35.0f

/* PID */
#define STEER_DEADBAND_DEG           0.3f
#define STEER_INT_ZONE_DEG           5.0f
#define STEER_KP                     200.0f
#define STEER_KI                     40.0f
#define STEER_KD                     8.0f

typedef struct
{
    float kp, ki, kd;
    float integral;
    float prev_error;
    float out_min, out_max;
    float int_min, int_max;
} PID_t;

static PID_t steer_pid =
{
    .kp         = STEER_KP,
    .ki         = STEER_KI,
    .kd         = STEER_KD,
    .integral   = 0.0f,
    .prev_error = 0.0f,
    .out_min    = -STEP_RATE_MAX,
    .out_max    =  STEP_RATE_MAX,
    .int_min    = -2000.0f,
    .int_max    =  2000.0f
};

/* Boot-zero references */
static float steering_cmd_ref_deg = -1.0f;
static float steering_fb_ref_deg  = -1.0f;

/* Working variables */
static float steering_cmd_deg    = 0.0f;
static float steering_fb_deg     = 0.0f;
static float steering_error_deg  = 0.0f;
static float step_rate_cmd       = 0.0f;
static float step_rate_out_left  = 0.0f;
static float step_rate_out_right = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C4_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM15_Init(void);

/* USER CODE BEGIN 0 */

/* ---- Math helpers ---- */
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

static float circular_mean_deg(float a_deg, float b_deg)
{
    float a_rad = a_deg * (float)M_PI / 180.0f;
    float b_rad = b_deg * (float)M_PI / 180.0f;
    float x = cosf(a_rad) + cosf(b_rad);
    float y = sinf(a_rad) + sinf(b_rad);
    return angle_wrap_360(atan2f(y, x) * 180.0f / (float)M_PI);
}

static float pid_update(PID_t *pid, float error, float dt)
{
    if ((error > 0.0f && pid->prev_error < 0.0f) ||
        (error < 0.0f && pid->prev_error > 0.0f))
    {
        pid->integral = 0.0f;
    }

    if (fabsf(error) < STEER_INT_ZONE_DEG)
    {
        pid->integral += error * pid->ki * dt;
        pid->integral  = clampf(pid->integral, pid->int_min, pid->int_max);
    }

    float derivative = (error - pid->prev_error) / dt;
    float output     = (pid->kp * error) + pid->integral + (pid->kd * derivative);
    output = clampf(output, pid->out_min, pid->out_max);
    pid->prev_error = error;
    return output;
}

/* ---- ADC helper ---- */
static uint16_t read_adc(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

/* ---- AS5600 helper ---- */
static float as5600_read_deg(I2C_HandleTypeDef *hi2c, uint8_t *buf)
{
    const uint8_t addr = (0x36 << 1);

    if (HAL_I2C_Mem_Read(hi2c, addr, 0x0E, I2C_MEMADD_SIZE_8BIT,
                         buf, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1.0f;
    }

    uint16_t raw = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    return (raw * 360.0f) / 4096.0f;
}

/* ---- Shared timer pulse writer ---- */
static void step_timer_set_rate(TIM_HandleTypeDef *htim, uint32_t channel, float rate_pps)
{
    if (rate_pps < STEP_RATE_MIN)
    {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
        htim->Instance->EGR = TIM_EGR_UG;
        return;
    }

    uint32_t arr = (uint32_t)((STEP_TIM_CLK_HZ / rate_pps) - 1.0f);

    if (arr < 10U)    arr = 10U;
    if (arr > 65535U) arr = 65535U;

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2U);
    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Instance->EGR = TIM_EGR_UG;
}

/* ================================================================
 * Left stepper (TIM8 CH1, PB6 PUL / PB4 DIR / PB5 ENA)
 * ================================================================ */
static void stepper_enable(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOB, EN_L_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void stepper_set_dir(float signed_rate)
{
    HAL_GPIO_WritePin(GPIOB, DIR_L_Pin,
        (signed_rate >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    htim8.Instance->EGR = TIM_EGR_UG;
    step_rate_out_left = 0.0f;
    stepper_enable(0);
}

static void steering_apply_output(float signed_rate_cmd)
{
    float delta = signed_rate_cmd - step_rate_out_left;

    if (delta >  STEP_RATE_SLEW_PER_TICK) delta =  STEP_RATE_SLEW_PER_TICK;
    if (delta < -STEP_RATE_SLEW_PER_TICK) delta = -STEP_RATE_SLEW_PER_TICK;

    step_rate_out_left += delta;
    step_rate_out_left = clampf(step_rate_out_left, -STEP_RATE_MAX, STEP_RATE_MAX);

    if (fabsf(step_rate_out_left) < STEP_RATE_MIN)
    {
        stepper_stop();
        return;
    }

    stepper_set_dir(step_rate_out_left);
    stepper_enable(1);
    step_timer_set_rate(&htim8, TIM_CHANNEL_1, fabsf(step_rate_out_left));
}

/* ================================================================
 * Right stepper (TIM15 CH2, PB3 PUL / PB10 DIR / PC3 ENA)
 * ================================================================ */
static void stepper_right_enable(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOC, EN_R_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void stepper_right_set_dir(float signed_rate)
{
    HAL_GPIO_WritePin(GPIOB, DIR_R_Pin,
        (signed_rate >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_right_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    htim15.Instance->EGR = TIM_EGR_UG;
    step_rate_out_right = 0.0f;
    stepper_right_enable(0);
}

static void steering_apply_output_right(float signed_rate_cmd)
{
    float delta = signed_rate_cmd - step_rate_out_right;

    if (delta >  STEP_RATE_SLEW_PER_TICK) delta =  STEP_RATE_SLEW_PER_TICK;
    if (delta < -STEP_RATE_SLEW_PER_TICK) delta = -STEP_RATE_SLEW_PER_TICK;

    step_rate_out_right += delta;
    step_rate_out_right = clampf(step_rate_out_right, -STEP_RATE_MAX, STEP_RATE_MAX);

    if (fabsf(step_rate_out_right) < STEP_RATE_MIN)
    {
        stepper_right_stop();
        return;
    }

    stepper_right_set_dir(step_rate_out_right);
    stepper_right_enable(1);
    step_timer_set_rate(&htim15, TIM_CHANNEL_2, fabsf(step_rate_out_right));
}

/* ---- Hall-sensor EXTI ISR ---- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t time_diff;
    current_time = __HAL_TIM_GET_COUNTER(&htim2);

    if (GPIO_Pin == GPIO_PIN_0)
    {
        time_diff = (current_time >= last_time_1)
                  ? (current_time - last_time_1)
                  : (0xFFFFFFFFU - last_time_1 + current_time);

        if (time_diff > 5000U)
        {
            rpm_1 = (60.0f * TICKS_PER_SECOND) /
                    ((float)time_diff * (float)PULSES_PER_REVOLUTION);
            rpm_1_buffer[rpm_1_buffer_index] = rpm_1;
            rpm_1_buffer_index = (rpm_1_buffer_index + 1) % AVG_SAMPLES;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SAMPLES; i++) sum += rpm_1_buffer[i];
            rpm_1_avg       = sum / (float)AVG_SAMPLES;
            last_time_1     = current_time;
            pulse_1_count++;
            new_rpm_1_ready = 1;
        }
    }
    else if (GPIO_Pin == GPIO_PIN_1)
    {
        time_diff = (current_time >= last_time_2)
                  ? (current_time - last_time_2)
                  : (0xFFFFFFFFU - last_time_2 + current_time);

        if (time_diff > 5000U)
        {
            rpm_2 = (60.0f * TICKS_PER_SECOND) /
                    ((float)time_diff * (float)PULSES_PER_REVOLUTION);
            rpm_2_buffer[rpm_2_buffer_index] = rpm_2;
            rpm_2_buffer_index = (rpm_2_buffer_index + 1) % AVG_SAMPLES;

            float sum = 0.0f;
            for (int i = 0; i < AVG_SAMPLES; i++) sum += rpm_2_buffer[i];
            rpm_2_avg       = sum / (float)AVG_SAMPLES;
            last_time_2     = current_time;
            pulse_2_count++;
            new_rpm_2_ready = 1;
        }
    }
}

/* USER CODE END 0 */

/**
 * @brief  Application entry point.
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C3_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_FDCAN1_Init();
    MX_I2C4_Init();
    MX_I2C2_Init();
    MX_TIM8_Init();
    MX_DAC1_Init();
    MX_TIM15_Init();

    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim2);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    stepper_enable(0);

    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    stepper_right_enable(0);

    HAL_I2C_Mem_Read (&hi2c3, 0xA6, 0x00, I2C_MEMADD_SIZE_8BIT, &adxl1_id,       1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x2D, I2C_MEMADD_SIZE_8BIT, &adxl_power_ctl, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x31, I2C_MEMADD_SIZE_8BIT, &adxl_power_ctl, 1, HAL_MAX_DELAY);

    HAL_I2C_Mem_Read (&hi2c3, 0x3A, 0x00, I2C_MEMADD_SIZE_8BIT, &adxl2_id,       1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x2D, I2C_MEMADD_SIZE_8BIT, &adxl_power_ctl, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x31, I2C_MEMADD_SIZE_8BIT, &adxl_power_ctl, 1, HAL_MAX_DELAY);

    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0x000;
    sFilterConfig.FilterID2    = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
            FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();

    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) Error_Handler();

    /* USER CODE END 2 */

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* --- Read all AS5600 sensors @ 500 Hz --- */
        if (now - last_tick_steering >= RATE_STEERING_MS)
        {
            float v;

            v = as5600_read_deg(&hi2c3, steering_data);
            if (v >= 0.0f) steering_deg = v;

            v = as5600_read_deg(&hi2c4, fb_left_data);
            if (v >= 0.0f) fb_left_deg = v;

            v = as5600_read_deg(&hi2c2, fb_right_data);
            if (v >= 0.0f) fb_right_deg = v;

            /* can_tx_steering1(steering_deg); */
            /* can_tx_steering2(fb_left_deg); */
            /* can_tx_steering3(fb_right_deg); */

            last_tick_steering = now;
        }

        /* --- Closed-loop steering control @ 500 Hz --- */
        if (now - last_tick_steering_ctrl >= RATE_STEERING_CTRL_MS)
        {
            const float dt = RATE_STEERING_CTRL_MS / 1000.0f;

            if (steering_cmd_ref_deg < 0.0f)
                steering_cmd_ref_deg = angle_wrap_360(steering_deg);

            if (steering_fb_ref_deg < 0.0f)
                steering_fb_ref_deg = circular_mean_deg(fb_left_deg, fb_right_deg);

            float wheel_delta = angle_error_deg_fn(
                                    angle_wrap_360(steering_deg),
                                    steering_cmd_ref_deg);

            steering_cmd_deg = clampf(
                (wheel_delta / WHEEL_DEG_RANGE) * RUDDER_DEG_MAX,
                RUDDER_DEG_MIN,
                RUDDER_DEG_MAX);

            float raw_fb = circular_mean_deg(fb_left_deg, fb_right_deg);

            steering_fb_deg = clampf(
                angle_error_deg_fn(raw_fb, steering_fb_ref_deg),
                RUDDER_DEG_MIN,
                RUDDER_DEG_MAX);

            steering_error_deg = steering_cmd_deg - steering_fb_deg;

            if (fabsf(steering_error_deg) < STEER_DEADBAND_DEG)
            {
                steering_error_deg = 0.0f;
                steer_pid.integral *= 0.90f;
                stepper_stop();
                stepper_right_stop();
            }
            else
            {
                step_rate_cmd = pid_update(&steer_pid, steering_error_deg, dt);
                steering_apply_output(step_rate_cmd);
                steering_apply_output_right(step_rate_cmd);
            }

            static uint32_t last_print = 0;
            if (now - last_print >= 100U)
            {
                printf("CMD=%.2f FB=%.2f ERR=%.2f RATE_L=%.1f RATE_R=%.1f\r\n",
                       steering_cmd_deg,
                       steering_fb_deg,
                       steering_error_deg,
                       step_rate_out_left,
                       step_rate_out_right);
                last_print = now;
            }

            last_tick_steering_ctrl = now;
        }

        /* --- Throttle @ 50 Hz --- */
        if (now - last_tick_throttle >= RATE_THROTTLE_MS)
        {
            throttle_raw = read_adc();
            throttle_out = (throttle_raw * 3.3f) / 4096.0f;
            /* can_tx_throttle(throttle_out); */
            last_tick_throttle = now;
        }

        /* --- ADXL345 #1 @ 100 Hz --- */
        if (now - last_tick_motor_vibrationLeft >= ADXL_SAMPLE_INTERVAL_MS)
        {
            HAL_I2C_Mem_Read(&hi2c3, 0xA6, 0x32, I2C_MEMADD_SIZE_8BIT,
                             adxl1_data, 6, HAL_MAX_DELAY);
            ax1 = (int16_t)((adxl1_data[1] << 8) | adxl1_data[0]);
            ay1 = (int16_t)((adxl1_data[3] << 8) | adxl1_data[2]);
            az1 = (int16_t)((adxl1_data[5] << 8) | adxl1_data[4]);

            if (adxl1_index < ADXL_SAMPLES)
            {
                accel_x1[adxl1_index] = ax1 * adxl_cal_val;
                accel_y1[adxl1_index] = ay1 * adxl_cal_val;
                accel_z1[adxl1_index] = az1 * adxl_cal_val;
                adxl1_index++;
            }
            else
            {
                mean_x1 = 0.0f; mean_y1 = 0.0f; mean_z1 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_x1 += accel_x1[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_y1 += accel_y1[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_z1 += accel_z1[i];
                mean_x1 /= ADXL_SAMPLES;
                mean_y1 /= ADXL_SAMPLES;
                mean_z1 /= ADXL_SAMPLES;

                sum_x1 = 0.0f; sum_y1 = 0.0f; sum_z1 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_x1 += (accel_x1[i] - mean_x1) * (accel_x1[i] - mean_x1);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_y1 += (accel_y1[i] - mean_y1) * (accel_y1[i] - mean_y1);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_z1 += (accel_z1[i] - mean_z1) * (accel_z1[i] - mean_z1);

                uint8_t motor_vibrationLeft_unsafe =
                    (sqrtf(sum_x1 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
                    (sqrtf(sum_y1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
                    (sqrtf(sum_z1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);

                (void)motor_vibrationLeft_unsafe;
                /* can_tx_adxl1(motor_vibrationLeft_unsafe); */

                adxl1_index = 0;
                sum_x1 = 0.0f; sum_y1 = 0.0f; sum_z1 = 0.0f;
                mean_x1 = 0.0f; mean_y1 = 0.0f; mean_z1 = 0.0f;
            }
            last_tick_motor_vibrationLeft = now;
        }

        /* --- ADXL345 #2 @ 100 Hz --- */
        if (now - last_tick_motor_vibrationRight >= ADXL_SAMPLE_INTERVAL_MS)
        {
            HAL_I2C_Mem_Read(&hi2c3, 0x3A, 0x32, I2C_MEMADD_SIZE_8BIT,
                             adxl2_data, 6, HAL_MAX_DELAY);
            ax2 = (int16_t)((adxl2_data[1] << 8) | adxl2_data[0]);
            ay2 = (int16_t)((adxl2_data[3] << 8) | adxl2_data[2]);
            az2 = (int16_t)((adxl2_data[5] << 8) | adxl2_data[4]);

            if (adxl2_index < ADXL_SAMPLES)
            {
                accel_x2[adxl2_index] = ax2 * adxl_cal_val;
                accel_y2[adxl2_index] = ay2 * adxl_cal_val;
                accel_z2[adxl2_index] = az2 * adxl_cal_val;
                adxl2_index++;
            }
            else
            {
                mean_x2 = 0.0f; mean_y2 = 0.0f; mean_z2 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_x2 += accel_x2[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_y2 += accel_y2[i];
                for (int i = 0; i < ADXL_SAMPLES; i++) mean_z2 += accel_z2[i];
                mean_x2 /= ADXL_SAMPLES;
                mean_y2 /= ADXL_SAMPLES;
                mean_z2 /= ADXL_SAMPLES;

                sum_x2 = 0.0f; sum_y2 = 0.0f; sum_z2 = 0.0f;
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_x2 += (accel_x2[i] - mean_x2) * (accel_x2[i] - mean_x2);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_y2 += (accel_y2[i] - mean_y2) * (accel_y2[i] - mean_y2);
                for (int i = 0; i < ADXL_SAMPLES; i++) sum_z2 += (accel_z2[i] - mean_z2) * (accel_z2[i] - mean_z2);

                uint8_t motor_vibrationRight_unsafe =
                    (sqrtf(sum_x2 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
                    (sqrtf(sum_y2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
                    (sqrtf(sum_z2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);

                (void)motor_vibrationRight_unsafe;
                /* can_tx_adxl2(motor_vibrationRight_unsafe); */

                adxl2_index = 0;
                sum_x2 = 0.0f; sum_y2 = 0.0f; sum_z2 = 0.0f;
                mean_x2 = 0.0f; mean_y2 = 0.0f; mean_z2 = 0.0f;
            }
            last_tick_motor_vibrationRight = now;
        }

        /* --- Hall #1 @ 1000 Hz --- */
        if (now - last_tick_motor_rpmLeft >= RATE_MOTOR_RPM_MS)
        {
            current_time = __HAL_TIM_GET_COUNTER(&htim2);
            uint32_t time_since = (current_time >= last_time_1)
                                ? (current_time - last_time_1)
                                : (0xFFFFFFFFU - last_time_1 + current_time);
            if (time_since > RPM_TIMEOUT)
            {
                rpm_1     = 0.0f;
                rpm_1_avg = 0.0f;
            }
            /* can_tx_rpm(rpm_1_avg); */
            last_tick_motor_rpmLeft = now;
        }

        /* --- Hall #2 @ 1000 Hz --- */
        if (now - last_tick_motor_rpmRight >= RATE_MOTOR_RPM_MS)
        {
            current_time = __HAL_TIM_GET_COUNTER(&htim2);
            uint32_t time_since = (current_time >= last_time_2)
                                ? (current_time - last_time_2)
                                : (0xFFFFFFFFU - last_time_2 + current_time);
            if (time_since > RPM_TIMEOUT)
            {
                rpm_2     = 0.0f;
                rpm_2_avg = 0.0f;
            }
            /* can_tx_rpm2(rpm_2_avg); */
            last_tick_motor_rpmRight = now;
        }
    }
}

/* ================================================================
 * System Clock Configuration
 * HSE -> PLL -> 170 MHz SYSCLK
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

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

    sConfig.Channel      = ADC_CHANNEL_7;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_DAC1_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) Error_Handler();

    sConfig.DAC_HighFrequency           = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode       = DISABLE;
    sConfig.DAC_SignedFormat            = DISABLE;
    sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2                = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
    sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) Error_Handler();
}

static void MX_FDCAN1_Init(void)
{
    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_FD_NO_BRS;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = ENABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 2;
    hfdcan1.Init.NominalSyncJumpWidth = 39;
    hfdcan1.Init.NominalTimeSeg1      = 130;
    hfdcan1.Init.NominalTimeSeg2      = 39;
    hfdcan1.Init.DataPrescaler        = 17;
    hfdcan1.Init.DataSyncJumpWidth    = 6;
    hfdcan1.Init.DataTimeSeg1         = 13;
    hfdcan1.Init.DataTimeSeg2         = 6;
    hfdcan1.Init.StdFiltersNbr        = 1;
    hfdcan1.Init.ExtFiltersNbr        = 0;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_QUEUE_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) Error_Handler();
}

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

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 169;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295UL;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim8.Instance               = TIM8;
    htim8.Init.Prescaler         = 169;
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
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim15.Instance               = TIM15;
    htim15.Init.Prescaler         = 169;
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

    GPIO_InitStruct.Pin  = Y3_AM_Pin | Y4_AM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = EN_R_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_R_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = Y2_AM_Pin | Y1_AM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = DIR_R_Pin | DIR_L_Pin | EN_L_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
