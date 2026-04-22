/* ================================================================
 * Stepper pulse config
 *
 * Target: 250 RPM output shaft
 *   Motor RPM  = 250 × 20 (gearbox) = 5000 RPM
 *   Motor RPS  = 5000 / 60 = 83.33 rev/s
 *   Pulse freq = 83.33 × 1600 = 133,333 Hz
 *
 * TIM8 / TIM15: APB2 @ 170 MHz, prescaler = 0
 *   ARR = 170,000,000 / 133,333 − 1 = 1274
 *   CMP = ARR / 2 = 637  → 50% duty cycle
 *
 * DM860I max pulse input = 200 kHz  → within spec
 * ================================================================ */

#define TIM_CLK_HZ          170000000UL

#define PULSE_FREQ_TARGET    133333.0f   // Hz → 250 RPM output
#define PULSE_FREQ_START       2000.0f   // Hz — safe cold-start frequency
#define PULSE_FREQ_MIN          500.0f   // Hz — below this, hold motor stopped
#define PULSE_FREQ_MAX       133333.0f   // Hz — hard cap (= PULSE_FREQ_TARGET)

/*
 * Trapezoidal ramp: how many Hz to add/subtract per control tick (2 ms).
 * 0 → 133,333 Hz in ~0.5 s  →  133333 / (0.5 / 0.002) = 533 Hz/tick
 * Tune RAMP_ACCEL_HZ_PER_TICK for faster/slower ramp.
 */
#define RAMP_ACCEL_HZ_PER_TICK   533.0f   // Hz per 2 ms tick → 0.5 s to full speed

/* Internal ramp state — one per axis */
static float freq_out_left  = 0.0f;   // current live pulse freq, left  motor
static float freq_out_right = 0.0f;   // current live pulse freq, right motor

/* ================================================================
 * Low-level timer helpers (shared logic, called for each axis)
 * ================================================================ */
static void _set_timer_freq(TIM_HandleTypeDef *htim, uint32_t channel,
                             float freq_hz, uint8_t is_advanced)
{
    if (freq_hz < PULSE_FREQ_MIN) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
        return;
    }
    uint32_t arr = (uint32_t)(TIM_CLK_HZ / freq_hz) - 1;
    if (arr < 10U)     arr = 10U;
    if (arr > 65535U)  arr = 65535U;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2);  // 50% duty
    /* Advanced timers (TIM8) need MOE enabled via BDTR — already set by CubeMX.
       Force an update event so ARR/CCR take effect immediately. */
    htim->Instance->EGR = TIM_EGR_UG;
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
    _set_timer_freq(&htim8, TIM_CHANNEL_1, 0.0f, 1);
    freq_out_left = 0.0f;
    stepper_enable(0);
}

/*
 * steering_apply_output — left axis
 *
 * signed_rate_cmd:  positive = forward, negative = reverse.
 *                   magnitude is interpreted as a *fraction* of
 *                   PULSE_FREQ_TARGET (output of PID, ±STEP_RATE_MAX).
 *
 * The trapezoidal ramp runs here every 2 ms control tick.
 * We ramp freq_out_left toward the target, then write to timer.
 */
static void steering_apply_output(float signed_rate_cmd)
{
    /* Map PID output (±STEP_RATE_MAX) → target pulse frequency */
    float target_freq = (fabsf(signed_rate_cmd) / STEP_RATE_MAX) * PULSE_FREQ_TARGET;
    target_freq = clampf(target_freq, 0.0f, PULSE_FREQ_MAX);

    if (target_freq < PULSE_FREQ_MIN) {
        /* Command is essentially zero — ramp down to stop */
        freq_out_left -= RAMP_ACCEL_HZ_PER_TICK;
        if (freq_out_left <= 0.0f) { stepper_stop(); return; }
    } else {
        /* Ramp toward target (accelerate or decelerate) */
        if (freq_out_left < PULSE_FREQ_START && target_freq > 0.0f)
            freq_out_left = PULSE_FREQ_START;   // jump to start freq, not 0
        float delta = target_freq - freq_out_left;
        if (delta >  RAMP_ACCEL_HZ_PER_TICK) delta =  RAMP_ACCEL_HZ_PER_TICK;
        if (delta < -RAMP_ACCEL_HZ_PER_TICK) delta = -RAMP_ACCEL_HZ_PER_TICK;
        freq_out_left += delta;
        freq_out_left  = clampf(freq_out_left, PULSE_FREQ_MIN, PULSE_FREQ_MAX);
    }

    /* Direction */
    HAL_GPIO_WritePin(GPIOB, DIR_L_Pin,
        signed_rate_cmd >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    stepper_enable(1);
    _set_timer_freq(&htim8, TIM_CHANNEL_1, freq_out_left, 1);
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
    _set_timer_freq(&htim15, TIM_CHANNEL_2, 0.0f, 0);
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
        if (freq_out_right < PULSE_FREQ_START && target_freq > 0.0f)
            freq_out_right = PULSE_FREQ_START;
        float delta = target_freq - freq_out_right;
        if (delta >  RAMP_ACCEL_HZ_PER_TICK) delta =  RAMP_ACCEL_HZ_PER_TICK;
        if (delta < -RAMP_ACCEL_HZ_PER_TICK) delta = -RAMP_ACCEL_HZ_PER_TICK;
        freq_out_right += delta;
        freq_out_right  = clampf(freq_out_right, PULSE_FREQ_MIN, PULSE_FREQ_MAX);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,
        signed_rate_cmd >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    stepper_right_enable(1);
    _set_timer_freq(&htim15, TIM_CHANNEL_2, freq_out_right, 0);
}
