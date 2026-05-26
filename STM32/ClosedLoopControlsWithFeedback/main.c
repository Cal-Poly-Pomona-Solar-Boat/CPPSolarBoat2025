/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @Authors: Tim and Daniel
 * @brief   Dual closed-loop stepper control - Solar Boat project
 *          Regen-safe: every custom edit lives inside a USER CODE block.
 *
 *   Control architecture:
 *
 *     AS5600 wheel (ADC5_IN1 / PA8) ── steering_tick() ──> shared target_deg
 *                                                              │
 *                                       ┌──────────────────────┴────────────────────┐
 *                                       ▼                                           ▼
 *                              LEFT P controller                          RIGHT P controller
 *                              err = target - enc_L                       err = target - enc_R
 *                                       │                                           │
 *                                       ▼                                           ▼
 *                              stepper_run(&stepper_left,  cmd_L)         stepper_run(&stepper_right, cmd_R)
 *
 *   Each motor closes the loop on its OWN encoder, so a stall or slip in
 *   one motor doesn't affect the other.  They share the setpoint from the
 *   AS5600 steering wheel.
 *
 *   Position-control API:
 *       stepper_set_target_deg() / stepper_move_deg()
 *       stepper_get_target_deg()
 *       stepper_get_left_deg()  / stepper_get_right_deg()
 *       stepper_at_target()  (both axes within deadband)
 *       stepper_position_tick()
 *
 *   Two encoders, one per motor:
 *       TIM3 (PA6/PA7)  -> LEFT  motor  -> encoder_left_get_deg()
 *       TIM5            -> RIGHT motor  -> encoder_right_get_deg()
 *   encoder_zero() captures both at startup.
 *
 *   Regen-safe overrides (do not move out of USER CODE blocks):
 *     - USER CODE BEGIN TIM2_Init 2  : PSC = STEP_TIM_PSC + EGR.UG
 *     - USER CODE BEGIN TIM8_Init 2  : PSC = STEP_TIM_PSC + EGR.UG
 *     - USER CODE BEGIN TIM15_Init 2 : PSC = STEP_TIM_PSC + EGR.UG
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    /* Hardware */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    uint8_t            invert_dir;

    /* Runtime state */
    uint8_t  enabled;
    uint8_t  enable_wait;
    uint32_t enable_tick;
    uint8_t  dir;
    uint8_t  dir_chg;
    uint32_t dir_tick;
    float    speed;
    float    rate_out;
} stepper_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Timer / pulse-rate base settings --- */
#define STEP_TIM_PSC               169U
#define STEP_TIM_CLK_HZ            1000000.0f
#define STEPS_PER_REV              3200UL

#define STEP_RATE_MAX              60000.0f
#define STEP_RATE_MIN              200.0f

#define RAMP_UP_SLEW               800.0f
#define RAMP_DOWN_SLEW             2000.0f
#define DIR_CHANGE_RAMP_HZ         2000.0f

#define STEPPER_ENABLE_SETTLE_MS   5U
#define DIR_CHANGE_SETTLE_MS       2U

#define RATE_STEPPER_MS            2U

/* --- Mechanical / position control --- */
#define GEAR_RATIO                 20.0f
#define STEPS_PER_OUT_DEG          ((float)STEPS_PER_REV * GEAR_RATIO / 360.0f)
#define POS_PEAK_PPS               RPM_TO_PPS(400.0f)
#define POS_KP                     30.0f
#define POS_DEADBAND_STEPS         250.0f  // ±1.5° of deadband

/* --- Encoder readback ---
 * One sign per encoder so each can be flipped independently on the bench. */
#define ENCODER_SIGN_LEFT          -1.0f
#define ENCODER_SIGN_RIGHT         -1.0f
#define ENCODER_COUNTS_PER_REV     4000U

/* --- AS5600 steering wheel input (analog on ADC5_IN1 / PA8) --- */
#define AS5600_SIGN                +1.0f
#define AS5600_FILT_ALPHA          0.15f
#define AS5600_OVERSAMPLE          8
#define AS5600_FAIL_LIMIT          5U

/* --- Steering wheel -> rudder mapping ---
 * STEER_RUDDER_RANGE_DEG : max rudder target in degrees (saturates).
 * STEER_GAIN             : output_deg per wheel_deg.  1.0 = 1:1.
 * STEER_TARGET_DEADBAND_DEG : if |wheel| < this, target = 0. */
#define STEER_RUDDER_RANGE_DEG     35.0f
#define STEER_GAIN                 0.5f  // 2:1 wheel:rudder
#define STEER_TARGET_DEADBAND_DEG  1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RPM_TO_PPS(rpm)    ((float)(rpm) * (float)STEPS_PER_REV / 60.0f)
#define PPS_TO_RPM(pps)    ((float)(pps) * 60.0f / (float)STEPS_PER_REV)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc5;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
stepper_t stepper_left = {
    .htim       = &htim8,
    .channel    = TIM_CHANNEL_1,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_L_Pin,
    .en_port    = GPIOB,
    .en_pin     = EN_L_Pin,
    .invert_dir = 0
};

stepper_t stepper_right = {
    .htim       = &htim15,
    .channel    = TIM_CHANNEL_2,
    .dir_port   = GPIOB,
    .dir_pin    = DIR_R_Pin,
    .en_port    = GPIOC,
    .en_pin     = EN_R_Pin,
    .invert_dir = 0
};

/* Loop scheduling */
static uint32_t last_tick   = 0;
static uint32_t last_print  = 0;

/* Encoder zero references - stored as raw uint16 counter snapshots so the
 * delta computation is wrap-correct via modular subtraction. */
static uint16_t encoder_left_zero_raw  = 0;
static uint16_t encoder_right_zero_raw = 0;

/* Position-control state.  Units = MOTOR steps.  One target shared by
 * both axes (both motors track the same wheel-derived angle), but each
 * axis has its own measured-position cache fed from its own encoder. */
static float    pos_target_steps        = 0.0f;
static float    pos_current_left_steps  = 0.0f;
static float    pos_current_right_steps = 0.0f;

/* AS5600 steering-wheel feedback state. */
static float    as5600_deg          = 0.0f;
static float    as5600_zero_deg     = -1.0f;
static uint8_t  as5600_ok           = 0U;
static uint16_t as5600_fail_count   = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM8_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC5_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void stepper_write_rate(stepper_t *s, float rate_pps);
static void stepper_write_dir(stepper_t *s, uint8_t positive);
static void stepper_write_enable(stepper_t *s, uint8_t enable);
void stepper_init(stepper_t *s);
void stepper_hard_stop(stepper_t *s);
void stepper_run(stepper_t *s, float cmd_pps);
void stepper_run_both(float cmd_pps);
void stepper_stop_both(void);

static void  encoder_zero(void);
static float encoder_left_get_deg(void);
static float encoder_right_get_deg(void);

void    stepper_set_target_deg(float output_deg);
void    stepper_move_deg(float output_deg_delta);
float   stepper_get_target_deg(void);
float   stepper_get_left_deg(void);
float   stepper_get_right_deg(void);
uint8_t stepper_at_target(void);
void    stepper_position_tick(void);

/* AS5600 steering-wheel input */
static float as5600_read_deg(void);
static void  as5600_tick(void);
float        as5600_get_wheel_deg(void);
void         as5600_zero_capture(void);
uint8_t      as5600_is_healthy(void);
void         steering_tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void stepper_write_rate(stepper_t *s, float rate_pps)
{
    if (rate_pps < STEP_RATE_MIN)
    {
        __HAL_TIM_SET_COMPARE(s->htim, s->channel, 0U);
        s->htim->Instance->EGR = TIM_EGR_UG;
        return;
    }

    if (rate_pps > STEP_RATE_MAX) rate_pps = STEP_RATE_MAX;

    uint32_t arr = (uint32_t)((STEP_TIM_CLK_HZ / rate_pps) - 1.0f);

    if (arr < 10U)    arr = 10U;
    if (arr > 65535U) arr = 65535U;

    __HAL_TIM_SET_AUTORELOAD(s->htim, arr);
    __HAL_TIM_SET_COMPARE(s->htim, s->channel, arr / 2U);
    s->htim->Instance->EGR = TIM_EGR_UG;
}

static void stepper_write_dir(stepper_t *s, uint8_t positive)
{
    uint8_t actual = s->invert_dir ? (uint8_t)(!positive) : positive;
    HAL_GPIO_WritePin(s->dir_port, s->dir_pin,
                      actual ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stepper_write_enable(stepper_t *s, uint8_t enable)
{
    HAL_GPIO_WritePin(s->en_port, s->en_pin,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void stepper_init(stepper_t *s)
{
    stepper_write_rate(s, 0.0f);
    stepper_write_enable(s, 0);

    s->enabled      = 0U;
    s->enable_wait  = 0U;
    s->enable_tick  = 0U;
    s->dir          = 1U;
    s->dir_chg      = 0U;
    s->dir_tick     = 0U;
    s->speed        = 0.0f;
    s->rate_out     = 0.0f;

    s->htim->Instance->BDTR |= TIM_BDTR_MOE;
}

void stepper_hard_stop(stepper_t *s)
{
    stepper_write_rate(s, 0.0f);
    stepper_write_enable(s, 0);

    s->enabled      = 0U;
    s->enable_wait  = 0U;
    s->speed        = 0.0f;
    s->rate_out     = 0.0f;
}

void stepper_run(stepper_t *s, float cmd_pps)
{
    uint32_t now      = HAL_GetTick();
    uint8_t  want_dir = (cmd_pps >= 0.0f) ? 1U : 0U;
    float    target   = fabsf(cmd_pps);

    if (target > STEP_RATE_MAX) target = STEP_RATE_MAX;

    if (s->enable_wait)
    {
        if ((now - s->enable_tick) < STEPPER_ENABLE_SETTLE_MS)
        {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        s->enable_wait = 0U;
        s->enabled     = 1U;
        if (s->speed < STEP_RATE_MIN) s->speed = STEP_RATE_MIN;
    }

    if (s->dir_chg)
    {
        if ((now - s->dir_tick) < DIR_CHANGE_SETTLE_MS)
        {
            stepper_write_rate(s, 0.0f);
            s->rate_out = 0.0f;
            return;
        }
        s->dir_chg = 0U;
        s->speed   = STEP_RATE_MIN;
    }

    if ((target >= STEP_RATE_MIN) && (want_dir != s->dir))
    {
        if (s->speed > DIR_CHANGE_RAMP_HZ)
        {
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed < DIR_CHANGE_RAMP_HZ) s->speed = DIR_CHANGE_RAMP_HZ;

            if (!s->enabled && !s->enable_wait)
            {
                stepper_write_dir(s, s->dir);
                stepper_write_enable(s, 1);
                s->enable_wait = 1U;
                s->enable_tick = now;
                s->rate_out    = 0.0f;
                return;
            }

            stepper_write_rate(s, s->speed);
            s->rate_out = s->dir ? s->speed : -s->speed;
            return;
        }

        stepper_write_rate(s, 0.0f);
        s->dir = want_dir;
        stepper_write_dir(s, s->dir);
        s->dir_chg  = 1U;
        s->dir_tick = now;
        s->speed    = 0.0f;
        s->rate_out = 0.0f;
        return;
    }

    if (target < STEP_RATE_MIN)
    {
        if (s->speed > 0.0f)
        {
            s->speed -= RAMP_DOWN_SLEW;
            if (s->speed <= STEP_RATE_MIN)
            {
                stepper_hard_stop(s);
                return;
            }
        }
        else
        {
            stepper_hard_stop(s);
            return;
        }
    }
    else if (s->speed < target)
    {
        s->speed += RAMP_UP_SLEW;
        if (s->speed > target) s->speed = target;
    }
    else if (s->speed > target)
    {
        s->speed -= RAMP_DOWN_SLEW;
        if (s->speed < target) s->speed = target;
        if (s->speed < STEP_RATE_MIN)
        {
            stepper_hard_stop(s);
            return;
        }
    }

    if (!s->enabled && !s->enable_wait)
    {
        stepper_write_dir(s, s->dir);
        stepper_write_enable(s, 1);
        s->enable_wait = 1U;
        s->enable_tick = now;
        s->rate_out    = 0.0f;
        return;
    }

    stepper_write_rate(s, s->speed);
    s->rate_out = s->dir ? s->speed : -s->speed;
}

void stepper_run_both(float cmd_pps)
{
    stepper_run(&stepper_left,  cmd_pps);
    stepper_run(&stepper_right, cmd_pps);
}

void stepper_stop_both(void)
{
    stepper_hard_stop(&stepper_left);
    stepper_hard_stop(&stepper_right);
}

/* ---------------------------------------------------------------------------
 * Encoder readback - one timer per motor in TIM_ENCODERMODE_TI12.
 *
 *   TIM3 quadrature on PA6/PA7  -> LEFT  motor
 *   TIM5 quadrature on its pins -> RIGHT motor
 *
 * Wrap-correct delta: we snapshot the raw uint16 counter at zero, then
 * each read subtracts uint16 - uint16 (modular) and casts the result to
 * int16 for the signed shortest path.  This gives correct deltas across
 * the 0x0000/0xFFFF boundary regardless of where the counter happened to
 * be at startup, as long as motion between calls stays under ±32767
 * quadrature edges (which is ±147 deg at output with our gear ratio -
 * far more than the rudder will ever move per call).
 * --------------------------------------------------------------------------- */
static void encoder_zero(void)
{
    encoder_left_zero_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    encoder_right_zero_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
}

static float encoder_left_get_deg(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t  delta = (int16_t)(now - encoder_left_zero_raw);
    return ENCODER_SIGN_LEFT *
           ((float)delta * 360.0f) /
           ((float)ENCODER_COUNTS_PER_REV * GEAR_RATIO);
}

static float encoder_right_get_deg(void)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    int16_t  delta = (int16_t)(now - encoder_right_zero_raw);
    return ENCODER_SIGN_RIGHT *
           ((float)delta * 360.0f) /
           ((float)ENCODER_COUNTS_PER_REV * GEAR_RATIO);
}

/* ---------------------------------------------------------------------------
 * Wrap-aware angle helpers (used by the AS5600 IIR filter so a step from
 * 359 -> 1 reads as +2 instead of -358).
 * --------------------------------------------------------------------------- */
static float angle_wrap_360(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg <    0.0f) deg += 360.0f;
    return deg;
}

static float angle_diff_deg(float target, float current)
{
    float d = target - current;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* ---------------------------------------------------------------------------
 * AS5600 analog read on ADC5_IN1 (PA8) - 0..3V3 = 0..360 deg.  Oversamples
 * AS5600_OVERSAMPLE times to knock down noise.  Returns -1.0 on error.
 * --------------------------------------------------------------------------- */
static float as5600_read_deg(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < AS5600_OVERSAMPLE; i++)
    {
        HAL_ADC_Start(&hadc5);
        if (HAL_ADC_PollForConversion(&hadc5, 5) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc5);
            return -1.0f;
        }
        sum += HAL_ADC_GetValue(&hadc5);
        HAL_ADC_Stop(&hadc5);
    }
    uint16_t avg = (uint16_t)(sum / AS5600_OVERSAMPLE);
    return ((float)avg * 360.0f) / 4096.0f;
}

static void as5600_tick(void)
{
    float v = as5600_read_deg();

    if (v >= 0.0f)
    {
        if (as5600_zero_deg < 0.0f)
        {
            /* First valid read: seed filter AND capture boot zero. */
            as5600_deg      = v;
            as5600_zero_deg = v;
        }
        else
        {
            float d = angle_diff_deg(v, as5600_deg);
            as5600_deg = angle_wrap_360(as5600_deg + AS5600_FILT_ALPHA * d);
        }
        as5600_ok         = 1U;
        as5600_fail_count = 0U;
    }
    else
    {
        if (as5600_fail_count < 0xFFFFU) as5600_fail_count++;
        if (as5600_fail_count >= AS5600_FAIL_LIMIT) as5600_ok = 0U;
    }
}

float as5600_get_wheel_deg(void)
{
    if (as5600_zero_deg < 0.0f) return 0.0f;
    return AS5600_SIGN * angle_diff_deg(as5600_deg, as5600_zero_deg);
}

void as5600_zero_capture(void)
{
    if (as5600_ok) as5600_zero_deg = as5600_deg;
}

uint8_t as5600_is_healthy(void)
{
    return as5600_ok;
}

/* ---------------------------------------------------------------------------
 * Steering tick - reads AS5600, runs button re-zero, computes the rudder
 * target with gain/deadband/clamp, pushes it into the position controller.
 * If the wheel sensor is unhealthy, target is FROZEN (rudder holds).
 * --------------------------------------------------------------------------- */
void steering_tick(void)
{
    as5600_tick();

    /* Optional runtime re-zero: hold the Zero_Steering button (PULLDOWN,
     * HIGH = pressed) while the wheel is centred to reset the logical zero. */
    if (HAL_GPIO_ReadPin(Zero_Steering_GPIO_Port, Zero_Steering_Pin) == GPIO_PIN_SET)
    {
        as5600_zero_capture();
    }

    if (!as5600_is_healthy()) return;

    float wheel  = as5600_get_wheel_deg();
    float target = wheel * STEER_GAIN;

    if (fabsf(target) < STEER_TARGET_DEADBAND_DEG) target = 0.0f;
    if (target >  STEER_RUDDER_RANGE_DEG) target =  STEER_RUDDER_RANGE_DEG;
    if (target < -STEER_RUDDER_RANGE_DEG) target = -STEER_RUDDER_RANGE_DEG;

    stepper_set_target_deg(target);
}

/* ---------------------------------------------------------------------------
 * Position control API - CLOSED-LOOP per axis.
 *
 *   stepper_set_target_deg(d)   absolute target in OUTPUT degrees (shared
 *                               by both motors)
 *   stepper_move_deg(d)         relative shift of target
 *   stepper_get_target_deg()    current target
 *   stepper_get_left_deg()      LEFT  measured position (encoder TIM3)
 *   stepper_get_right_deg()     RIGHT measured position (encoder TIM5)
 *   stepper_at_target()         1 when BOTH axes are within deadband
 *   stepper_position_tick()     run one control tick (every RATE_STEPPER_MS)
 *
 * Each motor runs an independent P controller against its OWN encoder.
 * That means if the left motor stalls, only the left motor sees high
 * error and gets pushed harder; the right motor isn't fighting against
 * the left's failure.  If they were sharing a single command (as in
 * stepper_run_both), a stalled left motor would still get a small
 * command because pos_current_steps would be near target.
 * --------------------------------------------------------------------------- */
void stepper_set_target_deg(float output_deg)
{
    pos_target_steps = output_deg * STEPS_PER_OUT_DEG;
}

void stepper_move_deg(float output_deg_delta)
{
    pos_target_steps += output_deg_delta * STEPS_PER_OUT_DEG;
}

float stepper_get_target_deg(void)
{
    return pos_target_steps / STEPS_PER_OUT_DEG;
}

float stepper_get_left_deg(void)
{
    return encoder_left_get_deg();
}

float stepper_get_right_deg(void)
{
    return encoder_right_get_deg();
}

uint8_t stepper_at_target(void)
{
    float err_l = pos_target_steps - pos_current_left_steps;
    float err_r = pos_target_steps - pos_current_right_steps;
    return (fabsf(err_l) < POS_DEADBAND_STEPS) &&
           (fabsf(err_r) < POS_DEADBAND_STEPS);
}

/* Per-axis P controller.  Called twice per tick - once per motor.
 * Reads THIS motor's encoder, computes THIS motor's error against the
 * shared target, commands ONLY this motor. */
static void axis_position_tick(stepper_t *s,
                               float    measured_deg,
                               float   *current_steps_out)
{
    *current_steps_out = measured_deg * STEPS_PER_OUT_DEG;

    float error = pos_target_steps - *current_steps_out;
    float cmd_pps;

    if (fabsf(error) < POS_DEADBAND_STEPS)
    {
        cmd_pps = 0.0f;
    }
    else
    {
        cmd_pps = error * POS_KP;
        if (cmd_pps >  POS_PEAK_PPS) cmd_pps =  POS_PEAK_PPS;
        if (cmd_pps < -POS_PEAK_PPS) cmd_pps = -POS_PEAK_PPS;
    }

    stepper_run(s, cmd_pps);
}

void stepper_position_tick(void)
{
    /* LEFT axis - closes on TIM3 encoder, commands only stepper_left. */
    axis_position_tick(&stepper_left,
                       encoder_left_get_deg(),
                       &pos_current_left_steps);

    /* RIGHT axis - closes on TIM5 encoder, commands only stepper_right. */
    axis_position_tick(&stepper_right,
                       encoder_right_get_deg(),
                       &pos_current_right_steps);
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
  MX_TIM8_Init();
  MX_DAC1_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  MX_ADC5_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);

  stepper_init(&stepper_left);
  stepper_init(&stepper_right);

  /* Start both encoder timers BEFORE encoder_zero() so both snapshots
   * are taken with the counters live. */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
      Error_Handler();
  if (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK)
      Error_Handler();
  encoder_zero();

  /* Try to prime the AS5600 so the boot-zero is captured before the
   * control loop runs.  Non-fatal if it doesn't respond - steering_tick()
   * will keep the target frozen at 0 until the sensor recovers. */
  for (int i = 0; i < 50 && !as5600_is_healthy(); i++)
  {
      as5600_tick();
      HAL_Delay(2);
  }

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  BSP_COM_Init(COM1, &BspCOMInit);

  printf("Solar Boat - dual closed-loop stepper control.\r\n");
  printf("STEPS_PER_REV=%lu  GEAR=%.1f:1  STEPS_PER_OUT_DEG=%.2f\r\n",
         STEPS_PER_REV, (double)GEAR_RATIO, (double)STEPS_PER_OUT_DEG);
  printf("POS_PEAK_PPS=%.0f (%.1f motor RPM)  POS_KP=%.1f  DEADBAND=%.0f steps\r\n",
         (double)POS_PEAK_PPS, (double)PPS_TO_RPM(POS_PEAK_PPS),
         (double)POS_KP, (double)POS_DEADBAND_STEPS);
  printf("Steering: range=+/-%.1f deg  gain=%.2f  deadband=%.1f deg  AS5600=%s\r\n",
         (double)STEER_RUDDER_RANGE_DEG, (double)STEER_GAIN,
         (double)STEER_TARGET_DEADBAND_DEG,
         as5600_is_healthy() ? "OK" : "MISSING");

  last_tick  = HAL_GetTick();
  last_print = HAL_GetTick();
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

      /* --- Control tick: wheel -> target -> per-motor closed-loop --- */
      if (now - last_tick >= RATE_STEPPER_MS)
      {
          steering_tick();          /* AS5600 wheel -> stepper_set_target_deg */
          stepper_position_tick();  /* each motor closes on its own encoder  */
          last_tick = now;
      }

      /* --- 10 Hz status: target vs each motor's measured position --- */
      if (now - last_print >= 100U)
      {
          printf("TGT=%+7.2f  ENC_L=%+7.2f  ENC_R=%+7.2f  err_L=%+7.2f  err_R=%+7.2f  at_tgt=%u\r\n",
                 (double)stepper_get_target_deg(),
                 (double)stepper_get_left_deg(),
                 (double)stepper_get_right_deg(),
                 (double)(stepper_get_target_deg() - stepper_get_left_deg()),
                 (double)(stepper_get_target_deg() - stepper_get_right_deg()),
                 stepper_at_target());
          last_print = now;
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
  * @brief ADC5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */

  /** Common config
  */
  hadc5.Instance = ADC5;
  hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc5.Init.Resolution = ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation = 0;
  hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait = DISABLE;
  hadc5.Init.ContinuousConvMode = DISABLE;
  hadc5.Init.NbrOfConversion = 1;
  hadc5.Init.DiscontinuousConvMode = DISABLE;
  hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc5.Init.DMAContinuousRequests = DISABLE;
  hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc5.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  /* USER CODE END ADC5_Init 2 */

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
  /* Force prescaler to STEP_TIM_PSC (1 MHz tick) regardless of what the
   * .ioc says.  PSC is shadowed - write it, then pulse EGR.UG to latch. */
  htim2.Instance->PSC = STEP_TIM_PSC;
  htim2.Instance->EGR = TIM_EGR_UG;
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* LEFT motor encoder.  PSC must stay 0 so the counter advances one tick
   * per quadrature edge.  Nothing to override here. */
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* RIGHT motor encoder.  PSC must stay 0 so the counter advances one tick
   * per quadrature edge.  Nothing to override here.
   * Note: same as TIM3, the input pins likely need pull-ups (set Pull-up
   * in the CubeMX GPIO tab on the assigned encoder pins) for open-collector
   * encoder outputs to register clean logic levels. */
  /* USER CODE END TIM5_Init 2 */

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
  htim8.Init.Prescaler = 169;
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
  /* Force prescaler to STEP_TIM_PSC (1 MHz tick).  CubeMX defaults this to
   * 0 which makes the timer tick at 170 MHz; the stepper rate math assumes
   * a 1 MHz tick, so without this override the pulses come out far too fast
   * for the DM860I to see and the motor is silent.  PSC is shadowed - we
   * write it then pulse EGR.UG so the next pulse comes out correctly. */
  htim8.Instance->PSC = STEP_TIM_PSC;
  htim8.Instance->EGR = TIM_EGR_UG;
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
  htim15.Init.Prescaler = 169;
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
  /* Force prescaler to STEP_TIM_PSC - same reason as TIM8_Init 2.  If the
   * .ioc currently has Prescaler=169 this is a no-op; if a future regen
   * resets it to 0, this line catches it.  PSC is shadowed, so EGR.UG. */
  htim15.Instance->PSC = STEP_TIM_PSC;
  htim15.Instance->EGR = TIM_EGR_UG;
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

  /*Configure GPIO pins : DIR_R_Pin DIR_L_Pin EN_L_Pin */
  GPIO_InitStruct.Pin = DIR_R_Pin|DIR_L_Pin|EN_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Zero_Steering_Pin */
  GPIO_InitStruct.Pin = Zero_Steering_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Zero_Steering_GPIO_Port, &GPIO_InitStruct);

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
