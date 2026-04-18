/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sensor Hub + FDCAN Transmit — STM32G474RE
  *
  * Sensors:
  *   - AS5600 #1 : magnetic steering angle (I2C3, addr 0x36) - Steering Input
  *   - AS5600 #2 : magnetic steering angle (I2C4, addr 0x36) - Left Rudder Position
  *   - AS5600 #3 : magnetic steering angle (I2C2, addr 0x36) - Right Rudder Position
  *   - ADXL345 #1 : motor 1 vibration      (I2C3, addr 0x53 / 0xA6)
  *   - ADXL345 #2 : motor 2 vibration      (I2C3, addr 0x1D / 0x3A)
  *   - Hall sensor #1 : motor RPM          (PA0, EXTI0)
  *   - Hall sensor #2 : motor RPM          (PA1, EXTI1)
  *   - Throttle : 0-5V mapping             (PC1, ADC1)
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

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim2;

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
#define RATE_STEPPER_FEEDBACK_MS    100U    //  10 Hz
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();
  MX_I2C4_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start(&htim2);

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

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

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
          printf("steering angle: %.2f deg\r\n", steering_deg);
          can_tx_steering1(steering_deg);
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
          printf("stepper_FeedbackLeft angle: %.2f deg\r\n", stepper_feedbackLeft_deg);
          can_tx_steering2(stepper_feedbackLeft_deg);
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
          printf("stepper_FeedbackRight angle: %.2f deg\r\n", stepper_feedbackRight_deg);
          can_tx_steering3(stepper_feedbackRight_deg);
          last_tick_stepper_feedbackRight = now;
      }

      /* -----------------------------------------------------------------
       * ID 0x2C8 — throttle_Input  (ADC1)  @ 50 Hz / 20 ms
       * ----------------------------------------------------------------- */
      if (now - last_tick_throttle >= RATE_THROTTLE_MS)
      {
          throttle_raw = read_adc();
          throttle_out = (throttle_raw * 3.3f) / 4096.0f;
          printf("throttle_Input: %.2f V\r\n", throttle_out);
          can_tx_throttle(throttle_out);
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
              printf("motor_VibrationLeft x:%0.2f y:%0.2f z:%0.2f\r\n",
                     accel_x1[adxl1_index], accel_y1[adxl1_index], accel_z1[adxl1_index]);
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

              printf("motor_VibrationLeft x deviation: %0.2f\r\n", sqrtf(sum_x1 / (float)ADXL_SAMPLES));
              printf("motor_VibrationLeft y deviation: %0.2f\r\n", sqrtf(sum_y1 / (float)ADXL_SAMPLES));
              printf("motor_VibrationLeft z deviation: %0.2f\r\n", sqrtf(sum_z1 / (float)ADXL_SAMPLES));
              printf("motor_VibrationLeft unsafe: %d\r\n", motor_vibrationLeft_unsafe);
              can_tx_adxl1(motor_vibrationLeft_unsafe);

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

              printf("motor_VibrationRight unsafe: %d\r\n", motor_vibrationRight_unsafe);
              can_tx_adxl2(motor_vibrationRight_unsafe);

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
          printf("motor_RpmLeft  Pulses: %lu  RPM: %.1f\r\n", pulse_1_count, rpm_1_avg);
          can_tx_rpm(rpm_1_avg);
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
          printf("motor_RpmRight Pulses: %lu  RPM: %.1f\r\n", pulse_2_count, rpm_2_avg);
          can_tx_rpm2(rpm_2_avg);
          last_tick_motor_rpmRight = now;
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
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

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef   multimode = {0};
  ADC_ChannelConfTypeDef sConfig   = {0};

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

/**
  * @brief FDCAN1 Initialization Function
  */
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

/**
  * @brief I2C2 Initialization Function
  */
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

/**
  * @brief I2C3 Initialization Function
  */
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

/**
  * @brief I2C4 Initialization Function
  */
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

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 169;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 4294967295;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
