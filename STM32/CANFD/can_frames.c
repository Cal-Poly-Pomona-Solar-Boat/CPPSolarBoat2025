/* ============================================================
 * can_frames.c
 * Builds and transmits all CAN FD frames.
 *
 * Each function:
 * 1. Receives sensor values as parameters from main.c
 * 2. Packs them into a byte array (big-endian)
 * 3. Transmits the frame via HAL_FDCAN_AddMessageToTxFifoQ
 * 4. Prints to UART if TX fails — does NOT call Error_Handler
 *    so the main loop always continues regardless of CAN status
 *
 * Encoding conventions:
 * - All multi-byte values are big-endian (high byte first)
 * - Floats with decimals are scaled to integers before packing:
 *   degrees  → x10   as uint16  (e.g. 270.5 → 2705)
 *   RPM      → x10   as uint16  (e.g. 123.4 → 1234)
 *   voltage  → x1000 as uint16  (e.g. 2.500 → 2500)
 * - Vibration uses a 1-byte Boolean flag (0=safe, 1=unsafe)
 * ============================================================ */

#include "can_frames.h"
#include "string.h"
#include "stdio.h"

/* ============================================================
 * TX headers — one per frame ID
 * ============================================================ */

static FDCAN_TxHeaderTypeDef TxHeader_0x040 = {
    .Identifier = 0x040, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x122 = {
    .Identifier = 0x122, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x123 = {
    .Identifier = 0x123, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x2C8 = {
    .Identifier = 0x2C8, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x420 = {
    .Identifier = 0x420, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x421 = {
    .Identifier = 0x421, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_2, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x480 = {
    .Identifier = 0x480, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_1, .FDFormat = FDCAN_FD_CAN
};

static FDCAN_TxHeaderTypeDef TxHeader_0x481 = {
    .Identifier = 0x481, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_1, .FDFormat = FDCAN_FD_CAN
};

/* --- Shared local buffers --- */
static uint8_t TxData_2B[2];
static uint8_t TxData_1B[1];

/* ----------------------------------------------------------
 * ID 0x040 — steering  (AS5600 #1, I2C3)  @ 500 Hz
 * ---------------------------------------------------------- */
void can_tx_steering1(float deg) {
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x040, TxData_2B) != HAL_OK)
        printf("TX 0x040 (steering) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x122 — stepper_FeedbackLeft  (AS5600 #2, I2C4)  @ 10 Hz
 * ---------------------------------------------------------- */
void can_tx_steering2(float deg) {
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x122, TxData_2B) != HAL_OK)
        printf("TX 0x122 (stepper_FeedbackLeft) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x123 — stepper_FeedbackRight  (AS5600 #3, I2C2)  @ 10 Hz
 * ---------------------------------------------------------- */
void can_tx_steering3(float deg) {
    uint16_t val = (uint16_t)(deg * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x123, TxData_2B) != HAL_OK)
        printf("TX 0x123 (stepper_FeedbackRight) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x2C8 — throttle_Input  (ADC1)  @ 50 Hz
 * ---------------------------------------------------------- */
void can_tx_throttle(float voltage) {
    uint16_t val = (uint16_t)(voltage * 1000.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x2C8, TxData_2B) != HAL_OK)
        printf("TX 0x2C8 (throttle_Input) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x420 — motor_RpmLeft  (Hall sensor #1)  @ 1000 Hz
 * ---------------------------------------------------------- */
void can_tx_rpm(float rpm) {
    uint16_t val = (uint16_t)(rpm * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x420, TxData_2B) != HAL_OK)
        printf("TX 0x420 (motor_RpmLeft) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x421 — motor_RpmRight  (Hall sensor #2)  @ 1000 Hz
 * ---------------------------------------------------------- */
void can_tx_rpm2(float rpm) {
    uint16_t val = (uint16_t)(rpm * 10.0f);
    TxData_2B[0] = (val >> 8) & 0xFF;
    TxData_2B[1] = val & 0xFF;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x421, TxData_2B) != HAL_OK)
        printf("TX 0x421 (motor_RpmRight) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x480 — motor_VibrationLeft  (ADXL345 #1)  @ 100 Hz
 * ---------------------------------------------------------- */
void can_tx_adxl1(uint8_t unsafe) {
    TxData_1B[0] = unsafe;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x480, TxData_1B) != HAL_OK)
        printf("TX 0x480 (motor_VibrationLeft) failed\r\n");
}

/* ----------------------------------------------------------
 * ID 0x481 — motor_VibrationRight  (ADXL345 #2)  @ 100 Hz
 * ---------------------------------------------------------- */
void can_tx_adxl2(uint8_t unsafe) {
    TxData_1B[0] = unsafe;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_0x481, TxData_1B) != HAL_OK)
        printf("TX 0x481 (motor_VibrationRight) failed\r\n");
}
