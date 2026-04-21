/* USER CODE BEGIN Header */
/* Gabo and Nathan
/**
******************************************************************************
* @file           : main.c
* @brief          : Sensor Hub + FDCAN Transmit — STM32G474RE
*
* Sensors:
*   - AS5600 #1 : magnetic steering angle (I2C3, addr 0x36)
*   - AS5600 #2 : magnetic steering angle (I2C4, addr 0x36)
*   - AS5600 #3 : magnetic steering angle (I2C2, addr 0x36)
*   - ADXL345 #1 : motor 1 vibration      (I2C3, addr 0x53 / 0xA6)
*   - ADXL345 #2 : motor 2 vibration      (I2C3, addr 0x1D / 0x3A)
*   - Hall sensor #1 : motor RPM          (PA0, EXTI0)
*   - Hall sensor #2 : motor RPM          (PA1, EXTI1)
*   - Throttle : 0-5V mapping             (PC0, ADC1)
*
* CAN FD TX frames (built and sent via can_frames.c):
*   ID 0x040  — steering              (AS5600 #1)  2 bytes   @ 500 Hz  (2ms)
*   ID 0x122  — stepper_FeedbackLeft  (AS5600 #2)  2 bytes   @  10 Hz  (100ms)
*   ID 0x123  — stepper_FeedbackRight (AS5600 #3)  2 bytes   @  10 Hz  (100ms)
*   ID 0x2C8  — throttle_Input                     2 bytes   @  50 Hz  (20ms)
*   ID 0x420  — motor_RpmLeft         (Hall #1)    2 bytes   @ 1000 Hz (1ms)
*   ID 0x421  — motor_RpmRight        (Hall #2)    2 bytes   @ 1000 Hz (1ms)
*   ID 0x480  — motor_VibrationLeft   (ADXL345 #1) 1 byte    @ 100 Hz  (10ms)
*   ID 0x481  — motor_VibrationRight  (ADXL345 #2) 1 byte    @ 100 Hz  (10ms)
*
* Timing (HAL_GetTick based, non-blocking):
*   throttle_Input        :  50 Hz →  20 ms
*   steering              : 500 Hz →   2 ms
*   stepper_FeedbackLeft  :  10 Hz → 100 ms
*   stepper_FeedbackRight :  10 Hz → 100 ms
*   motor_VibrationLeft   : 100 Hz →  10 ms  (ADXL sample interval = 10ms/10 = 1ms)
*   motor_VibrationRight  : 100 Hz →  10 ms  (ADXL sample interval = 10ms/10 = 1ms)
*   motor_RpmLeft         : 1000 Hz →  1 ms
*   motor_RpmRight        : 1000 Hz →  1 ms
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "can_frames.h"
#include "math.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
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
/* --- AS5600 #1  steering (I2C3) --- */
uint8_t  steering_data[2];
uint16_t steering_raw = 0;
float    steering_deg = 0.0f;
/* --- AS5600 #2  stepper_FeedbackLeft (I2C4) --- */
uint8_t  stepper_feedbackLeft_data[2];
uint16_t stepper_feedbackLeft_raw = 0;
float    stepper_feedbackLeft_deg = 0.0f;
/* --- AS5600 #3  stepper_FeedbackRight (I2C2) --- */
uint8_t  stepper_feedbackRight_data[2];
uint16_t stepper_feedbackRight_raw = 0;
float    stepper_feedbackRight_deg = 0.0f;
/* --- Throttle 0-5V (ADC1) --- */
uint16_t throttle_raw = 0;
float    throttle_out = 0.0f;
/* --- ADXL345 #1  motor_VibrationLeft --- */
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
/* --- ADXL345 #2  motor_VibrationRight --- */
uint8_t  adxl2_id;
uint8_t  adxl2_data[6];
int16_t  ax2, ay2, az2;
float    accel_x2[10], accel_y2[10], accel_z2[10];
float    mean_x2 = 0.0f, mean_y2 = 0.0f, mean_z2 = 0.0f;
float    sum_x2  = 0.0f, sum_y2  = 0.0f, sum_z2  = 0.0f;
int      adxl2_index = 0;
/* --- Hall-effect #1  motor_RpmLeft --- */
#define AVG_SAMPLES           5
#define PULSES_PER_REVOLUTION 4
#define RPM_TIMEOUT           3000000U    // 3 s in microseconds
#define TICKS_PER_SECOND      1000714.0f  // calibrated
volatile uint32_t current_time    = 0;
volatile uint32_t pulse_1_count     = 0;
volatile uint32_t last_time_1       = 0;
volatile float    rpm_1             = 0.0f;
volatile float    rpm_1_avg         = 0.0f;
volatile uint8_t  new_rpm_1_ready   = 0;
volatile float   rpm_1_buffer[AVG_SAMPLES] = {0};
volatile uint8_t rpm_1_buffer_index        = 0;
/* --- Hall-effect #2  motor_RpmRight --- */
volatile uint32_t pulse_2_count     = 0;
volatile uint32_t last_time_2       = 0;
volatile float    rpm_2             = 0.0f;
volatile float    rpm_2_avg         = 0.0f;
volatile uint8_t  new_rpm_2_ready   = 0;
volatile float   rpm_2_buffer[AVG_SAMPLES] = {0};
volatile uint8_t rpm_2_buffer_index        = 0;
/* --- FDCAN filter config --- */
FDCAN_FilterTypeDef sFilterConfig;
/* --- Send-rate intervals (ms) --- */
#define RATE_STEERING_MS              2U    // 500 Hz
#define RATE_STEPPER_FEEDBACK_MS      5U    //  10 Hz
#define RATE_THROTTLE_MS             20U    //  50 Hz
#define RATE_MOTOR_VIBRATION_MS      10U    // 100 Hz
#define RATE_MOTOR_RPM_MS             1U    // 1000 Hz
/* ADXL sample interval = vibration send-rate / number of samples */
#define ADXL_SAMPLE_INTERVAL_MS  (RATE_MOTOR_VIBRATION_MS / ADXL_SAMPLES)  // 1 ms
/* --- Tick timestamps — one per frame --- */
static uint32_t last_tick_steering              = 0;
static uint32_t last_tick_stepper_feedbackLeft  = 0;
static uint32_t last_tick_stepper_feedbackRight = 0;
static uint32_t last_tick_throttle              = 0;
static uint32_t last_tick_motor_vibrationLeft   = 0;
static uint32_t last_tick_motor_vibrationRight  = 0;
static uint32_t last_tick_motor_rpmLeft         = 0;
static uint32_t last_tick_motor_rpmRight        = 0;
/* ================= CLOSED-LOOP STEERING CONTROL ================= */
#define RATE_STEERING_CTRL_MS       2U
#define TIM8_CLK_HZ                 170000000.0f   // verify if needed
#define STEP_RATE_MAX               50000.0f       // max pulse frequency
#define STEP_RATE_MIN               150.0f         // below this = stop pulses
#define STEP_RATE_SLEW              10000.0f        // max change per control update
#define STEER_DEADBAND_DEG          0.4f
#define STEER_INT_ZONE_DEG          10.0f
/* PID gains: start here, then tune */
#define STEER_KP                    180.0f
#define STEER_KI                    6.0f
#define STEER_KD                    20.0f
typedef struct
{
 float kp;
 float ki;
 float kd;
 float integral;
 float prev_error;
 float out_min;
 float out_max;
 float int_min;
 float int_max;
} PID_t;
static PID_t steer_pid =
{
 .kp = STEER_KP,
 .ki = STEER_KI,
 .kd = STEER_KD,
 .integral = 0.0f,
 .prev_error = 0.0f,
 .out_min = -STEP_RATE_MAX,
 .out_max =  STEP_RATE_MAX,
 .int_min = -3000.0f,
 .int_max =  3000.0f
};
static uint32_t last_tick_steering_ctrl = 0;
static float steering_cmd_deg = 0.0f;
static float steering_fb_deg  = 0.0f;
static float steering_error_deg = 0.0f;
static float step_rate_cmd = 0.0f;
static float step_rate_out = 0.0f;
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
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---------- Right side driver helpers ---------- */
static void stepper_right_set_dir(float signed_rate)
{
   if (signed_rate >= 0.0f)
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
   else
       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
static void stepper_right_enable(uint8_t enable)
{
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static void stepper_right_stop(void)
{
   __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
   stepper_right_enable(0);
}
static void stepper_right_set_rate(float rate_abs)
{
   if (rate_abs < STEP_RATE_MIN)
   {
       stepper_right_stop();
       return;
   }
   uint32_t arr = (uint32_t)((TIM8_CLK_HZ / rate_abs) - 1.0f);
   if (arr < 100U) arr = 100U;
   if (arr > 65535U) arr = 65535U;
   __HAL_TIM_SET_AUTORELOAD(&htim15, arr);
   __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, arr / 2U);
   __HAL_TIM_SET_COUNTER(&htim15, 0);
   stepper_right_enable(1);
}
static void steering_apply_output_right(float signed_rate_cmd)
{
   float delta = signed_rate_cmd - step_rate_out_right;
   if (delta > STEP_RATE_SLEW)  delta = STEP_RATE_SLEW;
   if (delta < -STEP_RATE_SLEW) delta = -STEP_RATE_SLEW;
   step_rate_out_right += delta;
   if (fabsf(step_rate_out_right) < STEP_RATE_MIN)
   {
       stepper_right_stop();
       return;
   }
   stepper_right_set_dir(step_rate_out_right);
   stepper_right_set_rate(fabsf(step_rate_out_right));
}
/* Read ADC */
static uint16_t read_adc(void)
{
 HAL_ADC_Start(&hadc1);
 HAL_ADC_PollForConversion(&hadc1, 10);
 uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
 HAL_ADC_Stop(&hadc1);
 return v;
}
/* PA0/PA1 EXTI ISR — Hall sensor rising/falling edge for RPM */
static float clampf(float x, float min_val, float max_val)
{
 if (x < min_val) return min_val;
 if (x > max_val) return max_val;
 return x;
}
static float angle_wrap_360(float deg)
{
 while (deg >= 360.0f) deg -= 360.0f;
 while (deg < 0.0f) deg += 360.0f;
 return deg;
}
/* shortest signed angle difference: target - feedback */
static float angle_error_deg_fn(float target, float feedback)
{
 float err = target - feedback;
 while (err > 180.0f)  err -= 360.0f;
 while (err < -180.0f) err += 360.0f;
 return err;
}
/* circular average for two AS5600 feedback angles */
static float circular_mean_deg(float a_deg, float b_deg)
{
 float a_rad = a_deg * (float)M_PI / 180.0f;
 float b_rad = b_deg * (float)M_PI / 180.0f;
 float x = cosf(a_rad) + cosf(b_rad);
 float y = sinf(a_rad) + sinf(b_rad);
 float mean = atan2f(y, x) * 180.0f / (float)M_PI;
 return angle_wrap_360(mean);
}
static float pid_update(PID_t *pid, float error, float dt)
{
 float derivative;
 float output;
 if (fabsf(error) < STEER_INT_ZONE_DEG)
 {
     pid->integral += error * pid->ki * dt;
     pid->integral = clampf(pid->integral, pid->int_min, pid->int_max);
 }
 derivative = (error - pid->prev_error) / dt;
 output = (pid->kp * error) + pid->integral + (pid->kd * derivative);
 output = clampf(output, pid->out_min, pid->out_max);
 pid->prev_error = error;
 return output;
}
/* assumes PB5 LOW = enabled, HIGH = disabled */
static void stepper_enable(uint8_t enable)
{
 HAL_GPIO_WritePin(GPIOB, EN_L_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/* flip SET/RESET here if direction is backwards */
static void stepper_set_dir(float signed_rate)
{
 if (signed_rate >= 0.0f)
     HAL_GPIO_WritePin(GPIOB, DIR_L_Pin, GPIO_PIN_SET);
 else
     HAL_GPIO_WritePin(GPIOB, DIR_L_Pin, GPIO_PIN_RESET);
}
static void stepper_stop(void)
{
 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
 stepper_enable(0);
}
static void stepper_set_rate(float rate_abs)
{
 if (rate_abs < STEP_RATE_MIN)
 {
     stepper_stop();
     return;
 }
 uint32_t arr = (uint32_t)((TIM8_CLK_HZ / rate_abs) - 1.0f);
 if (arr < 100U) arr = 100U;
 if (arr > 65535U) arr = 65535U;
 __HAL_TIM_SET_AUTORELOAD(&htim8, arr);
 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, arr / 10U);
 __HAL_TIM_SET_COUNTER(&htim8, 0);
 stepper_enable(1);
}
static void steering_apply_output(float signed_rate_cmd)
{
 float delta = signed_rate_cmd - step_rate_out;
 if (delta > STEP_RATE_SLEW) delta = STEP_RATE_SLEW;
 if (delta < -STEP_RATE_SLEW) delta = -STEP_RATE_SLEW;
 step_rate_out += delta;
 if (fabsf(step_rate_out) < STEP_RATE_MIN)
 {
     stepper_stop();
     return;
 }
 stepper_set_dir(step_rate_out);
 stepper_set_rate(fabsf(step_rate_out));
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 uint32_t time_diff_1;
 uint32_t time_diff_2;
 current_time = __HAL_TIM_GET_COUNTER(&htim2);
 if (GPIO_Pin == GPIO_PIN_0)
 {
     if (current_time >= last_time_1)
         time_diff_1 = current_time - last_time_1;
     else
         time_diff_1 = (0xFFFFFFFFU - last_time_1) + current_time;
     if (time_diff_1 > 5000U)
     {
         rpm_1 = (60.0f * TICKS_PER_SECOND) /
               ((float)time_diff_1 * (float)PULSES_PER_REVOLUTION);
         rpm_1_buffer[rpm_1_buffer_index] = rpm_1;
         rpm_1_buffer_index = (rpm_1_buffer_index + 1) % AVG_SAMPLES;
         float sum_1 = 0.0f;
         for (int i = 0; i < AVG_SAMPLES; i++)
             sum_1 += rpm_1_buffer[i];
         rpm_1_avg = sum_1 / (float)AVG_SAMPLES;
         last_time_1     = current_time;
         pulse_1_count++;
         new_rpm_1_ready = 1;
     }
 }
 else if (GPIO_Pin == GPIO_PIN_1)
 {
     if (current_time >= last_time_2)
         time_diff_2 = current_time - last_time_2;
     else
         time_diff_2 = (0xFFFFFFFFU - last_time_2) + current_time;
     if (time_diff_2 > 5000U)
     {
         rpm_2 = (60.0f * TICKS_PER_SECOND) /
               ((float)time_diff_2 * (float)PULSES_PER_REVOLUTION);
         rpm_2_buffer[rpm_2_buffer_index] = rpm_2;
         rpm_2_buffer_index = (rpm_2_buffer_index + 1) % AVG_SAMPLES;
         float sum_2 = 0.0f;
         for (int i = 0; i < AVG_SAMPLES; i++)
             sum_2 += rpm_2_buffer[i];
         rpm_2_avg = sum_2 / (float)AVG_SAMPLES;
         last_time_2     = current_time;
         pulse_2_count++;
         new_rpm_2_ready = 1;
     }
 }
}
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
 /* USER CODE BEGIN 1 */
 /* USER CODE END 1 */
 /* MCU Configuration--------------------------------------------------------*/
 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 HAL_Init();
 /* USER CODE BEGIN Init */
 /* USER CODE END Init */
 /* Configure the system clock */
 SystemClock_Config();
 /* USER CODE BEGIN SysInit */
 /* USER CODE END SysInit */
 /* Initialize all configured peripherals */
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
 /* Start PWM for left-side DM860I pulse output on PB6 / TIM8 CH1 */
 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
 stepper_enable(0);
 /* Start PWM for right-side DM860I pulse output on PB3 / TIM2 CH2 */
 HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
 __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
 stepper_right_enable(0);
 /* --- ADXL345 #1 (addr 0xA6) --- */
 HAL_I2C_Mem_Read (&hi2c3, 0xA6, 0x00, 1, &adxl1_id,       1, HAL_MAX_DELAY);
 HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x2D, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
 HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x31, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
 /* --- ADXL345 #2 (addr 0x3A) --- */
 HAL_I2C_Mem_Read (&hi2c3, 0x3A, 0x00, 1, &adxl2_id,       1, HAL_MAX_DELAY);
 HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x2D, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
 HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x31, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
 /* --- FDCAN filter --- */
 sFilterConfig.IdType       = FDCAN_STANDARD_ID;
 sFilterConfig.FilterIndex  = 0;
 sFilterConfig.FilterType   = FDCAN_FILTER_DUAL;
 sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
 sFilterConfig.FilterID1    = 0x000;
 sFilterConfig.FilterID2    = 0x000;
 if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
     Error_Handler();
 if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
         FDCAN_REJECT, FDCAN_REJECT,
         FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
     Error_Handler();
 if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
     Error_Handler();
 /* --- UART console --- */
 BspCOMInit.BaudRate   = 115200;
 BspCOMInit.WordLength = COM_WORDLENGTH_8B;
 BspCOMInit.StopBits   = COM_STOPBITS_1;
 BspCOMInit.Parity     = COM_PARITY_NONE;
 BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
 if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
     Error_Handler();
 /* USER CODE END 2 */
 /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
 BspCOMInit.BaudRate   = 115200;
 BspCOMInit.WordLength = COM_WORDLENGTH_8B;
 BspCOMInit.StopBits   = COM_STOPBITS_1;
 BspCOMInit.Parity     = COM_PARITY_NONE;
 BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
 if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
 {
   Error_Handler();
 }
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
while (1)
{
   uint32_t now = HAL_GetTick();
   /* -----------------------------------------------------------------
    * ID 0x040 — steering  (AS5600 #1, I2C3)  @ 500 Hz / 2 ms
    * ----------------------------------------------------------------- */
   if (now - last_tick_steering >= RATE_STEERING_MS)
   {
       if (HAL_I2C_Mem_Read(&hi2c3, 0x36 << 1, 0x0E,
               I2C_MEMADD_SIZE_8BIT, &steering_data[0], 1, HAL_MAX_DELAY) == HAL_OK
        && HAL_I2C_Mem_Read(&hi2c3, 0x36 << 1, 0x0F,
               I2C_MEMADD_SIZE_8BIT, &steering_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
       {
           steering_raw = ((uint16_t)steering_data[0] << 8) | steering_data[1];
           steering_deg = (steering_raw * 360.0f) / 4096.0f;
       }
      // printf("steering angle: %.2f deg\r\n", steering_deg);
      // can_tx_steering1(steering_deg);
       last_tick_steering = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x122 — stepper_FeedbackLeft  (AS5600 #2, I2C4)  @ 10 Hz / 100 ms
    * ----------------------------------------------------------------- */
   if (now - last_tick_stepper_feedbackLeft >= RATE_STEPPER_FEEDBACK_MS)
   {
       if (HAL_I2C_Mem_Read(&hi2c4, 0x36 << 1, 0x0E,
               I2C_MEMADD_SIZE_8BIT, &stepper_feedbackLeft_data[0], 1, HAL_MAX_DELAY) == HAL_OK
        && HAL_I2C_Mem_Read(&hi2c4, 0x36 << 1, 0x0F,
               I2C_MEMADD_SIZE_8BIT, &stepper_feedbackLeft_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
       {
           stepper_feedbackLeft_raw = ((uint16_t)stepper_feedbackLeft_data[0] << 8) | stepper_feedbackLeft_data[1];
           stepper_feedbackLeft_deg = (stepper_feedbackLeft_raw * 360.0f) / 4096.0f;
       }
      // printf("stepper_FeedbackLeft angle: %.2f deg\r\n", stepper_feedbackLeft_deg);
      // can_tx_steering2(stepper_feedbackLeft_deg);
       last_tick_stepper_feedbackLeft = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x123 — stepper_FeedbackRight  (AS5600 #3, I2C2)  @ 10 Hz / 100 ms
    * ----------------------------------------------------------------- */
   if (now - last_tick_stepper_feedbackRight >= RATE_STEPPER_FEEDBACK_MS)
   {
       if (HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0E,
               I2C_MEMADD_SIZE_8BIT, &stepper_feedbackRight_data[0], 1, HAL_MAX_DELAY) == HAL_OK
        && HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0F,
               I2C_MEMADD_SIZE_8BIT, &stepper_feedbackRight_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
       {
           stepper_feedbackRight_raw = ((uint16_t)stepper_feedbackRight_data[0] << 8) | stepper_feedbackRight_data[1];
           stepper_feedbackRight_deg = (stepper_feedbackRight_raw * 360.0f) / 4096.0f;
       }
      // printf("stepper_FeedbackRight angle: %.2f deg\r\n", stepper_feedbackRight_deg);
      // can_tx_steering3(stepper_feedbackRight_deg);
       last_tick_stepper_feedbackRight = now;
   }
   /* -----------------------------------------------------------------
    * Closed-loop steering control using AS5600 sensors only
    *
    * AS5600 #1 = steering command
    * AS5600 #2/#3 = steering actuator feedback
    * PB4 = DIR
    * PB5 = ENA
    * PB6 = PUL (TIM8 PWM)
    * ----------------------------------------------------------------- */
   if (now - last_tick_steering_ctrl >= RATE_STEERING_CTRL_MS)
   {
       const float dt = RATE_STEERING_CTRL_MS / 1000.0f;
       /* command from AS5600 #1 */
       steering_cmd_deg = angle_wrap_360(steering_deg);
       /* feedback from AS5600 #2 and #3 */
       steering_fb_deg = circular_mean_deg(stepper_feedbackLeft_deg, stepper_feedbackRight_deg);
       /* wrapped position error */
       steering_error_deg = angle_error_deg_fn(steering_cmd_deg, steering_fb_deg);
       /* deadband */
       if (fabsf(steering_error_deg) < STEER_DEADBAND_DEG)
       {
           steering_error_deg = 0.0f;
       }
       /* PID output -> signed step pulse rate */
       step_rate_cmd = pid_update(&steer_pid, steering_error_deg, dt);
       /* apply to left DM860I */
       steering_apply_output(step_rate_cmd);
       /* apply same command to right DM860I */
       steering_apply_output_right(step_rate_cmd);
       static uint32_t last_ctrl_print = 0;
       if (now - last_ctrl_print > 100)
       {
      	 printf("CTRL cmd=%.2f fb=%.2f err=%.2f rate=%.1f\r\n",
              steering_cmd_deg,
              steering_fb_deg,
              steering_error_deg,
              step_rate_out);
       last_ctrl_print = now;
   }
   last_tick_steering_ctrl = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x2C8 — throttle_Input  (ADC1)  @ 50 Hz / 20 ms
    * ----------------------------------------------------------------- */
   if (now - last_tick_throttle >= RATE_THROTTLE_MS)
   {
       throttle_raw = read_adc();
       throttle_out = (throttle_raw * 3.3f) / 4096.0f;
      // printf("throttle_Input: %.2f V\r\n", throttle_out);
       //can_tx_throttle(throttle_out);
        last_tick_throttle = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x480 — motor_VibrationLeft  (ADXL345 #1)  @ 100 Hz / 10 ms
    * Samples collected every ADXL_SAMPLE_INTERVAL_MS (1 ms)
    * CAN frame sent once per ADXL_SAMPLES (10) samples with unsafe flag
    * ----------------------------------------------------------------- */
   if (now - last_tick_motor_vibrationLeft >= ADXL_SAMPLE_INTERVAL_MS)
   {
       HAL_I2C_Mem_Read(&hi2c3, 0xA6, 0x32, 1, adxl1_data, 6, HAL_MAX_DELAY);
       ax1 = (int16_t)((adxl1_data[1] << 8) | adxl1_data[0]);
       ay1 = (int16_t)((adxl1_data[3] << 8) | adxl1_data[2]);
       az1 = (int16_t)((adxl1_data[5] << 8) | adxl1_data[4]);
       if (adxl1_index < ADXL_SAMPLES)
       {
           accel_x1[adxl1_index] = ax1 * adxl_cal_val;
           accel_y1[adxl1_index] = ay1 * adxl_cal_val;
           accel_z1[adxl1_index] = az1 * adxl_cal_val;
           // printf("motor_VibrationLeft x:%0.2f y:%0.2f z:%0.2f\r\n",
                 // accel_x1[adxl1_index], accel_y1[adxl1_index], accel_z1[adxl1_index]);
           adxl1_index++;
       }
       else
       {
           for (int i = 0; i < adxl1_index; i++) mean_x1 += accel_x1[i];
           for (int i = 0; i < adxl1_index; i++) mean_y1 += accel_y1[i];
           for (int i = 0; i < adxl1_index; i++) mean_z1 += accel_z1[i];
           mean_x1 /= ADXL_SAMPLES;
           mean_y1 /= ADXL_SAMPLES;
           mean_z1 /= ADXL_SAMPLES;
           for (int i = 0; i < adxl1_index; i++) sum_x1 += (accel_x1[i] - mean_x1) * (accel_x1[i] - mean_x1);
           for (int i = 0; i < adxl1_index; i++) sum_y1 += (accel_y1[i] - mean_y1) * (accel_y1[i] - mean_y1);
           for (int i = 0; i < adxl1_index; i++) sum_z1 += (accel_z1[i] - mean_z1) * (accel_z1[i] - mean_z1);
           uint8_t motor_vibrationLeft_unsafe =
               (sqrtf(sum_x1 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
               (sqrtf(sum_y1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
               (sqrtf(sum_z1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);
          // printf("motor_VibrationLeft x deviation: %0.2f\r\n", sqrtf(sum_x1 / (float)ADXL_SAMPLES));
           //printf("motor_VibrationLeft y deviation: %0.2f\r\n", sqrtf(sum_y1 / (float)ADXL_SAMPLES));
           //printf("motor_VibrationLeft z deviation: %0.2f\r\n", sqrtf(sum_z1 / (float)ADXL_SAMPLES));
          // printf("motor_VibrationLeft unsafe: %d\r\n", motor_vibrationLeft_unsafe);
           //can_tx_adxl1(motor_vibrationLeft_unsafe);
           adxl1_index = 0;
           sum_x1  = 0.0f; sum_y1  = 0.0f; sum_z1  = 0.0f;
           mean_x1 = 0.0f; mean_y1 = 0.0f; mean_z1 = 0.0f;
       }
       last_tick_motor_vibrationLeft = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x481 — motor_VibrationRight  (ADXL345 #2)  @ 100 Hz / 10 ms
    * Same approach as motor_VibrationLeft
    * ----------------------------------------------------------------- */
   if (now - last_tick_motor_vibrationRight >= ADXL_SAMPLE_INTERVAL_MS)
   {
       HAL_I2C_Mem_Read(&hi2c3, 0x3A, 0x32, 1, adxl2_data, 6, HAL_MAX_DELAY);
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
           for (int i = 0; i < adxl2_index; i++) mean_x2 += accel_x2[i];
           for (int i = 0; i < adxl2_index; i++) mean_y2 += accel_y2[i];
           for (int i = 0; i < adxl2_index; i++) mean_z2 += accel_z2[i];
           mean_x2 /= ADXL_SAMPLES;
           mean_y2 /= ADXL_SAMPLES;
           mean_z2 /= ADXL_SAMPLES;
           for (int i = 0; i < adxl2_index; i++) sum_x2 += (accel_x2[i] - mean_x2) * (accel_x2[i] - mean_x2);
           for (int i = 0; i < adxl2_index; i++) sum_y2 += (accel_y2[i] - mean_y2) * (accel_y2[i] - mean_y2);
           for (int i = 0; i < adxl2_index; i++) sum_z2 += (accel_z2[i] - mean_z2) * (accel_z2[i] - mean_z2);
           uint8_t motor_vibrationRight_unsafe =
               (sqrtf(sum_x2 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
               (sqrtf(sum_y2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
               (sqrtf(sum_z2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);
          // printf("motor_VibrationRight unsafe: %d\r\n", motor_vibrationRight_unsafe);
          // can_tx_adxl2(motor_vibrationRight_unsafe);
           adxl2_index = 0;
           sum_x2  = 0.0f; sum_y2  = 0.0f; sum_z2  = 0.0f;
           mean_x2 = 0.0f; mean_y2 = 0.0f; mean_z2 = 0.0f;
       }
       last_tick_motor_vibrationRight = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x420 — motor_RpmLeft  (Hall sensor #1)  @ 1000 Hz / 1 ms
    * ----------------------------------------------------------------- */
   if (now - last_tick_motor_rpmLeft >= RATE_MOTOR_RPM_MS)
   {
       current_time = __HAL_TIM_GET_COUNTER(&htim2);
       uint32_t time_since_pulse_1;
       if (current_time >= last_time_1)
           time_since_pulse_1 = current_time - last_time_1;
       else
           time_since_pulse_1 = (0xFFFFFFFFU - last_time_1) + current_time;
       if (time_since_pulse_1 > RPM_TIMEOUT)
       {
           rpm_1     = 0.0f;
           rpm_1_avg = 0.0f;
       }
       //printf("motor_RpmLeft  Pulses: %lu  RPM: %.1f\r\n", pulse_1_count, rpm_1_avg);
      // can_tx_rpm(rpm_1_avg);
       last_tick_motor_rpmLeft = now;
   }
   /* -----------------------------------------------------------------
    * ID 0x421 — motor_RpmRight  (Hall sensor #2)  @ 1000 Hz / 1 ms
    * ----------------------------------------------------------------- */
   if (now - last_tick_motor_rpmRight >= RATE_MOTOR_RPM_MS)
   {
       current_time = __HAL_TIM_GET_COUNTER(&htim2);
       uint32_t time_since_pulse_2;
       if (current_time >= last_time_2)
           time_since_pulse_2 = current_time - last_time_2;
       else
           time_since_pulse_2 = (0xFFFFFFFFU - last_time_2) + current_time;
       if (time_since_pulse_2 > RPM_TIMEOUT)
       {
           rpm_2     = 0.0f;
           rpm_2_avg = 0.0f;
       }
       //printf("motor_RpmRight Pulses: %lu  RPM: %.1f\r\n", pulse_2_count, rpm_2_avg);
       //can_tx_rpm2(rpm_2_avg);
       last_tick_motor_rpmRight = now;
   }
   /* USER CODE END WHILE */
   /* USER CODE BEGIN 3 */
}
 /* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
 /** Configure the main internal regulator output voltage
 */
 HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_ON;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
 RCC_OscInitStruct.PLL.PLLN = 85;
 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
 RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
 RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
   Error_Handler();
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
 {
   Error_Handler();
 }
}
/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
 /* USER CODE BEGIN ADC1_Init 0 */
 /* USER CODE END ADC1_Init 0 */
 ADC_MultiModeTypeDef multimode = {0};
 ADC_ChannelConfTypeDef sConfig = {0};
 /* USER CODE BEGIN ADC1_Init 1 */
 /* USER CODE END ADC1_Init 1 */
 /** Common config
 */
 hadc1.Instance = ADC1;
 hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
 hadc1.Init.Resolution = ADC_RESOLUTION_12B;
 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
 hadc1.Init.GainCompensation = 0;
 hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
 hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
 hadc1.Init.LowPowerAutoWait = DISABLE;
 hadc1.Init.ContinuousConvMode = DISABLE;
 hadc1.Init.NbrOfConversion = 1;
 hadc1.Init.DiscontinuousConvMode = DISABLE;
 hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
 hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
 hadc1.Init.DMAContinuousRequests = DISABLE;
 hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
 hadc1.Init.OversamplingMode = DISABLE;
 if (HAL_ADC_Init(&hadc1) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure the ADC multi-mode
 */
 multimode.Mode = ADC_MODE_INDEPENDENT;
 if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Regular Channel
 */
 sConfig.Channel = ADC_CHANNEL_7;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
 sConfig.SingleDiff = ADC_SINGLE_ENDED;
 sConfig.OffsetNumber = ADC_OFFSET_NONE;
 sConfig.Offset = 0;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN ADC1_Init 2 */
 /* USER CODE END ADC1_Init 2 */
}
/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{
 /* USER CODE BEGIN DAC1_Init 0 */
 /* USER CODE END DAC1_Init 0 */
 DAC_ChannelConfTypeDef sConfig = {0};
 /* USER CODE BEGIN DAC1_Init 1 */
 /* USER CODE END DAC1_Init 1 */
 /** DAC Initialization
 */
 hdac1.Instance = DAC1;
 if (HAL_DAC_Init(&hdac1) != HAL_OK)
 {
   Error_Handler();
 }
 /** DAC channel OUT1 config
 */
 sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
 sConfig.DAC_DMADoubleDataMode = DISABLE;
 sConfig.DAC_SignedFormat = DISABLE;
 sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
 sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
 sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
 sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
 sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
 sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
 if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
 {
   Error_Handler();
 }
 /** DAC channel OUT2 config
 */
 if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN DAC1_Init 2 */
 /* USER CODE END DAC1_Init 2 */
}
/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{
 /* USER CODE BEGIN FDCAN1_Init 0 */
 /* USER CODE END FDCAN1_Init 0 */
 /* USER CODE BEGIN FDCAN1_Init 1 */
 /* USER CODE END FDCAN1_Init 1 */
 hfdcan1.Instance = FDCAN1;
 hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
 hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
 hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
 hfdcan1.Init.AutoRetransmission = ENABLE;
 hfdcan1.Init.TransmitPause = ENABLE;
 hfdcan1.Init.ProtocolException = DISABLE;
 hfdcan1.Init.NominalPrescaler = 2;
 hfdcan1.Init.NominalSyncJumpWidth = 39;
 hfdcan1.Init.NominalTimeSeg1 = 130;
 hfdcan1.Init.NominalTimeSeg2 = 39;
 hfdcan1.Init.DataPrescaler = 17;
 hfdcan1.Init.DataSyncJumpWidth = 6;
 hfdcan1.Init.DataTimeSeg1 = 13;
 hfdcan1.Init.DataTimeSeg2 = 6;
 hfdcan1.Init.StdFiltersNbr = 1;
 hfdcan1.Init.ExtFiltersNbr = 0;
 hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
 if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN FDCAN1_Init 2 */
 /* USER CODE END FDCAN1_Init 2 */
}
/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{
 /* USER CODE BEGIN I2C2_Init 0 */
 /* USER CODE END I2C2_Init 0 */
 /* USER CODE BEGIN I2C2_Init 1 */
 /* USER CODE END I2C2_Init 1 */
 hi2c2.Instance = I2C2;
 hi2c2.Init.Timing = 0x40B285C2;
 hi2c2.Init.OwnAddress1 = 0;
 hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
 hi2c2.Init.OwnAddress2 = 0;
 hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
 hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
 hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c2) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analogue filter
 */
 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Digital filter
 */
 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN I2C2_Init 2 */
 /* USER CODE END I2C2_Init 2 */
}
/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{
 /* USER CODE BEGIN I2C3_Init 0 */
 /* USER CODE END I2C3_Init 0 */
 /* USER CODE BEGIN I2C3_Init 1 */
 /* USER CODE END I2C3_Init 1 */
 hi2c3.Instance = I2C3;
 hi2c3.Init.Timing = 0x40B285C2;
 hi2c3.Init.OwnAddress1 = 0;
 hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
 hi2c3.Init.OwnAddress2 = 0;
 hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
 hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
 hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c3) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analogue filter
 */
 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Digital filter
 */
 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN I2C3_Init 2 */
 /* USER CODE END I2C3_Init 2 */
}
/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void)
{
 /* USER CODE BEGIN I2C4_Init 0 */
 /* USER CODE END I2C4_Init 0 */
 /* USER CODE BEGIN I2C4_Init 1 */
 /* USER CODE END I2C4_Init 1 */
 hi2c4.Instance = I2C4;
 hi2c4.Init.Timing = 0x40B285C2;
 hi2c4.Init.OwnAddress1 = 0;
 hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
 hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
 hi2c4.Init.OwnAddress2 = 0;
 hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
 hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
 hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
 if (HAL_I2C_Init(&hi2c4) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Analogue filter
 */
 if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
 {
   Error_Handler();
 }
 /** Configure Digital filter
 */
 if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN I2C4_Init 2 */
 /* USER CODE END I2C4_Init 2 */
}
/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
 /* USER CODE BEGIN TIM2_Init 0 */
 /* USER CODE END TIM2_Init 0 */
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 /* USER CODE BEGIN TIM2_Init 1 */
 /* USER CODE END TIM2_Init 1 */
 htim2.Instance = TIM2;
 htim2.Init.Prescaler = 169;
 htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim2.Init.Period = 4294967295;
 htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
 {
   Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
 {
   Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN TIM2_Init 2 */
 /* USER CODE END TIM2_Init 2 */
}
/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{
 /* USER CODE BEGIN TIM8_Init 0 */
 /* USER CODE END TIM8_Init 0 */
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 TIM_OC_InitTypeDef sConfigOC = {0};
 TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
 /* USER CODE BEGIN TIM8_Init 1 */
 /* USER CODE END TIM8_Init 1 */
 htim8.Instance = TIM8;
 htim8.Init.Prescaler = 0;
 htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim8.Init.Period = 65535;
 htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim8.Init.RepetitionCounter = 0;
 htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
 {
   Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
 {
   Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
 {
   Error_Handler();
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 0;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
 sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
 {
   Error_Handler();
 }
 sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
 sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
 sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
 sBreakDeadTimeConfig.DeadTime = 0;
 sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
 sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
 sBreakDeadTimeConfig.BreakFilter = 0;
 sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
 sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
 sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
 sBreakDeadTimeConfig.Break2Filter = 0;
 sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
 sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
 if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN TIM8_Init 2 */
 /* USER CODE END TIM8_Init 2 */
 HAL_TIM_MspPostInit(&htim8);
}
/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{
 /* USER CODE BEGIN TIM15_Init 0 */
 /* USER CODE END TIM15_Init 0 */
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 TIM_OC_InitTypeDef sConfigOC = {0};
 TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
 /* USER CODE BEGIN TIM15_Init 1 */
 /* USER CODE END TIM15_Init 1 */
 htim15.Instance = TIM15;
 htim15.Init.Prescaler = 0;
 htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim15.Init.Period = 65535;
 htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim15.Init.RepetitionCounter = 0;
 htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
 {
   Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
 {
   Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
 {
   Error_Handler();
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 0;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
 sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
 {
   Error_Handler();
 }
 sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
 sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
 sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
 sBreakDeadTimeConfig.DeadTime = 0;
 sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
 sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
 sBreakDeadTimeConfig.BreakFilter = 0;
 sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
 if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN TIM15_Init 2 */
 /* USER CODE END TIM15_Init 2 */
 HAL_TIM_MspPostInit(&htim15);
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 /* USER CODE BEGIN MX_GPIO_Init_1 */
 /* USER CODE END MX_GPIO_Init_1 */
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOF_CLK_ENABLE();
 __HAL_RCC_GPIOC_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(EN_R_GPIO_Port, EN_R_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, DIR_R_Pin|DIR_L_Pin|EN_L_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pins : Y3_AM_Pin Y4_AM_Pin */
 GPIO_InitStruct.Pin = Y3_AM_Pin|Y4_AM_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 /*Configure GPIO pin : EN_R_Pin */
 GPIO_InitStruct.Pin = EN_R_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(EN_R_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pins : PA0 PA1 */
 GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
 GPIO_InitStruct.Pull = GPIO_PULLUP;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 /*Configure GPIO pins : Y2_AM_Pin Y1_AM_Pin */
 GPIO_InitStruct.Pin = Y2_AM_Pin|Y1_AM_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 /*Configure GPIO pins : DIR_R_Pin DIR_L_Pin EN_L_Pin */
 GPIO_InitStruct.Pin = DIR_R_Pin|DIR_L_Pin|EN_L_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 /* EXTI interrupt init*/
 HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI0_IRQn);
 HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
 HAL_NVIC_EnableIRQ(EXTI1_IRQn);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
 /* USER CODE BEGIN Error_Handler_Debug */
__disable_irq();
while (1) {}
 /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
 /* USER CODE BEGIN 6 */
 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

