/* USER CODE BEGIN Header */
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
  *   - Throttle : 0-5V mapping				(PC0, ADC1)
  *
  * CAN FD TX frames (built and sent via can_frames.c):
  *   ID 0x000  — AS5600 #1 steering angle     2 bytes
  *   ID 0x001  — dummy frame                  2 bytes
  *   ID 0x002  — ADXL345 #1 (motor 1)         8 bytes (6 data + 2 pad)
  *   ID 0x003  — ADXL345 #2 (motor 2)         8 bytes (6 data + 2 pad)
  *   ID 0x004  — Hall sensor RPM              2 bytes
  *   ID 0x005  — Quadrature encoder           4 bytes
  *   ID 0x006  — AS5600 #2 steering angle     2 bytes
  *
  * Timing:
  *   All frames currently send at 1000ms (HAL_GetTick based, non-blocking)
  *   To change a rate later, add a new #define and swap SENSOR_RATE_MS
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

/* --- AS5600 #1 steering angle (I2C3) --- */
uint8_t  as56001_data[2];
uint16_t as56001_raw = 0;
float    as56001_deg = 0.0f;

/* --- AS5600 #2 steering angle (I2C4) --- */
uint8_t  as56002_data[2];
uint16_t as56002_raw = 0;
float    as56002_deg = 0.0f;

/* --- AS5600 #3 steering angle (I2C2) --- */
uint8_t  as56003_data[2];
uint16_t as56003_raw = 0;
float    as56003_deg = 0.0f;

/* --- Throttle 0-5V (ADC1) --- */
uint16_t throttle_raw = 0;
float    throttle_out = 0.0f;

/* --- ADXL345 #1 (motor 1) --- */
#define ADXL_SAMPLES 10
#define ADXL_THRESH_X 0.08f
#define ADXL_THRESH_Y 0.08f
#define ADXL_THRESH_Z 0.08f

uint8_t  adxl1_id;
uint8_t  adxl1_data[6];
int16_t  ax1, ay1, az1;
float	 accel_x1[10], accel_y1[10], accel_z1[10];
float	 mean_x1 = 0.0f, mean_y1 = 0.0f, mean_z1 = 0.0f;
float	 sum_x1 = 0.0f, sum_y1 = 0.0f, sum_z1 = 0.0f;
uint8_t  adxl_power_ctl = 0x08;
float    adxl_cal_val   = 0.0039f;
int		 adxl1_index = 0;

/* --- ADXL345 #2 (motor 2) --- */
uint8_t  adxl2_id;
uint8_t  adxl2_data[6];
int16_t  ax2, ay2, az2;
float	 accel_x2[10], accel_y2[10], accel_z2[10];
float	 mean_x2 = 0.0f, mean_y2 = 0.0f, mean_z2 = 0.0f;
float	 sum_x2 = 0.0f, sum_y2 = 0.0f, sum_z2 = 0.0f;
int		 adxl2_index = 0;

/* --- Hall-effect #1 RPM --- */
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

/* --- Hall-effect #2 RPM --- */
volatile uint32_t pulse_2_count     = 0;
volatile uint32_t last_time_2       = 0;
volatile float    rpm_2             = 0.0f;
volatile float    rpm_2_avg         = 0.0f;
volatile uint8_t  new_rpm_2_ready   = 0;
volatile float   rpm_2_buffer[AVG_SAMPLES] = {0};
volatile uint8_t rpm_2_buffer_index        = 0;

/* --- FDCAN filter config --- */
FDCAN_FilterTypeDef sFilterConfig;

/* --- Sensor update rate (ms) ---
 * All sensors share the same rate for now.
 * To set different rates later, add one #define per sensor
 * and swap SENSOR_RATE_MS in that block. */
#define SENSOR_RATE_MS   1000U

/* --- Tick timestamps — one per frame ---
 * Initialised to 0, first update fires immediately on boot */
static uint32_t last_tick_steering1 = 0;
static uint32_t last_tick_steering2 = 0;
static uint32_t last_tick_steering3 = 0;
static uint32_t last_tick_throttle 	= 0;
static uint32_t last_tick_dummy     = 0;
static uint32_t last_tick_adxl1     = 0;
static uint32_t last_tick_adxl2     = 0;
static uint32_t last_tick_rpm_1     = 0;
static uint32_t last_tick_rpm_2     = 0;

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

/* PA0 EXTI ISR — Hall sensor rising/falling edge for RPM */
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

        if (time_diff_1 > 5000U) // >5 ms debounce
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

		if (time_diff_2 > 5000U) // >5 ms debounce
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
  /* USER CODE BEGIN 2 */

    /* --- Free-running microsecond timer --- */
    HAL_TIM_Base_Start(&htim2);

    /* --- ADXL345 #1 (addr 0xA6) --- */
    HAL_I2C_Mem_Read (&hi2c3, 0xA6, 0x00, 1, &adxl1_id,       1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x2D, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0xA6, 0x31, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);

    /* --- ADXL345 #2 (addr 0x3A) --- */
    HAL_I2C_Mem_Read (&hi2c3, 0x3A, 0x00, 1, &adxl2_id,       1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x2D, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c3, 0x3A, 0x31, 1, &adxl_power_ctl, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);

    /* --- FDCAN filter (required even for TX-only) --- */
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

    /* --- Start FDCAN --- */
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
       * ID 0x000 — AS5600 #1 steering angle (I2C3)
       * ----------------------------------------------------------------- */
      if (now - last_tick_steering1 >= SENSOR_RATE_MS)
      {
          if (HAL_I2C_Mem_Read(&hi2c3, 0x36 << 1, 0x0E,
                  I2C_MEMADD_SIZE_8BIT, &as56001_data[0], 1, HAL_MAX_DELAY) == HAL_OK
           && HAL_I2C_Mem_Read(&hi2c3, 0x36 << 1, 0x0F,
                  I2C_MEMADD_SIZE_8BIT, &as56001_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
          {
              as56001_raw = ((uint16_t)as56001_data[0] << 8) | as56001_data[1];
              as56001_deg = (as56001_raw * 360.0f) / 4096.0f;
          }
          // printf("AS5600 #1 angle: %.2f deg\r\n", as56001_deg);
          can_tx_steering1(as56001_deg);
          last_tick_steering1 = now;
      }

      /* -----------------------------------------------------------------
       * ID 0x006 — AS5600 #2 steering angle (I2C4)
       * ----------------------------------------------------------------- */
      if (now - last_tick_steering2 >= SENSOR_RATE_MS)
      {
          if (HAL_I2C_Mem_Read(&hi2c4, 0x36 << 1, 0x0E,
                  I2C_MEMADD_SIZE_8BIT, &as56002_data[0], 1, HAL_MAX_DELAY) == HAL_OK
           && HAL_I2C_Mem_Read(&hi2c4, 0x36 << 1, 0x0F,
                  I2C_MEMADD_SIZE_8BIT, &as56002_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
          {
              as56002_raw = ((uint16_t)as56002_data[0] << 8) | as56002_data[1];
              as56002_deg = (as56002_raw * 360.0f) / 4096.0f;
          }
          // printf("AS5600 #2 angle: %.2f deg\r\n", as56002_deg);
          can_tx_steering2(as56002_deg);
          last_tick_steering2 = now;
      }

      /* -----------------------------------------------------------------
       * ID INSERT ID — AS5600 #3 steering angle (I2C2)
       * ----------------------------------------------------------------- */
      if (now - last_tick_steering3 >= SENSOR_RATE_MS)
      {
          if (HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0E,
                  I2C_MEMADD_SIZE_8BIT, &as56003_data[0], 1, HAL_MAX_DELAY) == HAL_OK
           && HAL_I2C_Mem_Read(&hi2c2, 0x36 << 1, 0x0F,
                  I2C_MEMADD_SIZE_8BIT, &as56003_data[1], 1, HAL_MAX_DELAY) == HAL_OK)
          {
              as56003_raw = ((uint16_t)as56003_data[0] << 8) | as56003_data[1];
              as56003_deg = (as56003_raw * 360.0f) / 4096.0f;
          }
          // printf("AS5600 #3 angle: %.2f deg\r\n", as56003_deg);
          //can_tx_steering3(as56003_deg);
          last_tick_steering3 = now;
      }

      /* -----------------------------------------------------------------
       * ID INSERT ID — Throttle 0-5V (ADC)
       * ----------------------------------------------------------------- */
      if (now - last_tick_throttle >= SENSOR_RATE_MS)
      {
          throttle_raw = read_adc();
          throttle_out = (throttle_raw * 5.0f) / 4096.0f;
          // printf("Throttle: %0.2f \r\n", throttle_out);
          // can_tx_(throttle_raw);
          last_tick_throttle = now;
      }

      /* -----------------------------------------------------------------
       * ID 0x001 — dummy frame
       * ----------------------------------------------------------------- */
      if (now - last_tick_dummy >= SENSOR_RATE_MS)
      {
          can_tx_dummy();
          last_tick_dummy = now;
      }

      /* -----------------------------------------------------------------
       * ID 0x002 — ADXL345 #1 (motor 1 vibration)
       * ----------------------------------------------------------------- */
      if (now - last_tick_adxl1 >= SENSOR_RATE_MS / ADXL_SAMPLES)
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
			  printf("ADXL1 x:%0.2f y:%0.2f z:%0.2f\r\n", accel_x1[adxl1_index], accel_y1[adxl1_index], accel_z1[adxl1_index]);
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

        	  uint8_t adxl1_unsafe = (sqrtf(sum_x1 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
        			  	  	  	     (sqrtf(sum_y1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
								     (sqrtf(sum_z1 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);
              printf("ADXL1 x deviation: %0.2f\r\n", sqrtf(sum_x1 / (float)ADXL_SAMPLES));
              printf("ADXL1 y deviation: %0.2f\r\n", sqrtf(sum_y1 / (float)ADXL_SAMPLES));
              printf("ADXL1 z deviation: %0.2f\r\n", sqrtf(sum_z1 / (float)ADXL_SAMPLES));
        	  printf("ADXL1 unsafe?: %d\r\n", adxl1_unsafe);

        	  //can_tx_adxl1(adxl1_unsafe);
        	  adxl1_index = 0;
			  sum_x1 = 0.0f;
			  sum_y1 = 0.0f;
			  sum_z1 = 0.0f;
			  mean_x1 = 0.0f;
			  mean_y1 = 0.0f;
			  mean_z1 = 0.0f;
          }

		  last_tick_adxl1 = now;
      }

      /* -----------------------------------------------------------------
       * ID 0x003 — ADXL345 #2 (motor 2 vibration)
       * ----------------------------------------------------------------- */
      if (now - last_tick_adxl2 >= SENSOR_RATE_MS / ADXL_SAMPLES)
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

        	  uint8_t adxl2_unsafe = (sqrtf(sum_x2 / (float)ADXL_SAMPLES) > ADXL_THRESH_X) ||
        			  	  	  	   	 (sqrtf(sum_y2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Y) ||
								     (sqrtf(sum_z2 / (float)ADXL_SAMPLES) > ADXL_THRESH_Z);
              printf("ADXL2 unsafe?: %d\r\n", adxl2_unsafe);

        	  //can_tx_adxl2(adxl2_unsafe);
        	  adxl2_index = 0;
			  sum_x2 = 0.0f;
			  sum_y2 = 0.0f;
			  sum_z2 = 0.0f;
			  mean_x2 = 0.0f;
			  mean_y2 = 0.0f;
			  mean_z2 = 0.0f;
          }

		  last_tick_adxl2 = now;
      }

      /* -----------------------------------------------------------------
       * ID 0x004 — Hall sensor #1 RPM
       * ----------------------------------------------------------------- */
      if (now - last_tick_rpm_1 >= SENSOR_RATE_MS)
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
          // printf("RPM #1 Pulses: %lu  RPM: %.1f\r\n", pulse_1_count, rpm_1_avg);
          //can_tx_rpm(rpm_1_avg);
          last_tick_rpm_1 = now;
      }

      /* -----------------------------------------------------------------
       * ID INSERT ID — Hall sensor #2 RPM
       * ----------------------------------------------------------------- */
      if (now - last_tick_rpm_2 >= SENSOR_RATE_MS)
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
         // printf("RPM #2 Pulses: %lu  RPM: %.1f\r\n", pulse_2_count, rpm_2_avg);
         //can_tx_rpm(rpm_2_avg);
         last_tick_rpm_2 = now;
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
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
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

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  while (1)
  {
  }
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
