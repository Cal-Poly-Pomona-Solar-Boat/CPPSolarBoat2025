#ifndef CAN_FRAMES_H
#define CAN_FRAMES_H

#include "main.h"

/* ============================================================
 * can_frames.h
 * Declarations for all CAN FD TX frame functions.
 *
 * Each function takes sensor values as parameters,
 * builds the payload, and transmits the frame.
 *
 * Frame ID map:
 *   0x040  — steering              (AS5600 #1)   2 bytes  @ 500 Hz
 *   0x122  — stepper_FeedbackLeft  (AS5600 #2)   2 bytes  @  10 Hz
 *   0x123  — stepper_FeedbackRight (AS5600 #3)   2 bytes  @  10 Hz
 *   0x2C8  — throttle_Input        (ADC1)         2 bytes  @  50 Hz
 *   0x420  — motor_RpmLeft         (Hall #1)      2 bytes  @ 1000 Hz
 *   0x421  — motor_RpmRight        (Hall #2)      2 bytes  @ 1000 Hz
 *   0x480  — motor_VibrationLeft   (ADXL345 #1)  1 byte   @ 100 Hz
 *   0x481  — motor_VibrationRight  (ADXL345 #2)  1 byte   @ 100 Hz
 * ============================================================ */

/* --- FDCAN handle — defined in main.c, used here --- */
extern FDCAN_HandleTypeDef hfdcan1;

/* ----------------------------------------------------------
 * ID 0x040 — steering  (AS5600 #1, I2C3)  @ 500 Hz
 * Parameters:
 *   deg : steering angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 *   Example: 270.5° → 2705 → [0x0A, 0x91]
 * ---------------------------------------------------------- */
void can_tx_steering1(float deg);

/* ----------------------------------------------------------
 * ID 0x122 — stepper_FeedbackLeft  (AS5600 #2, I2C4)  @ 10 Hz
 * Parameters:
 *   deg : angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering2(float deg);

/* ----------------------------------------------------------
 * ID 0x123 — stepper_FeedbackRight  (AS5600 #3, I2C2)  @ 10 Hz
 * Parameters:
 *   deg : angle in degrees (0.0 to 360.0)
 * Payload: deg * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_steering3(float deg);

/* ----------------------------------------------------------
 * ID 0x2C8 — throttle_Input  (ADC1)  @ 50 Hz
 * Parameters:
 *   voltage : throttle voltage (0.0 to 3.3)
 * Payload: voltage * 1000 as uint16, big-endian
 *   Example: 2.500V → 2500 → [0x09, 0xC4]
 * ---------------------------------------------------------- */
void can_tx_throttle(float voltage);

/* ----------------------------------------------------------
 * ID 0x420 — motor_RpmLeft  (Hall sensor #1)  @ 1000 Hz
 * Parameters:
 *   rpm : averaged RPM value (0.0 to 9999.9)
 * Payload: rpm * 10 as uint16, big-endian
 *   Example: 123.4 RPM → 1234 → [0x04, 0xD2]
 * ---------------------------------------------------------- */
void can_tx_rpm(float rpm);

/* ----------------------------------------------------------
 * ID 0x421 — motor_RpmRight  (Hall sensor #2)  @ 1000 Hz
 * Parameters:
 *   rpm : averaged RPM value (0.0 to 9999.9)
 * Payload: rpm * 10 as uint16, big-endian
 * ---------------------------------------------------------- */
void can_tx_rpm2(float rpm);

/* ----------------------------------------------------------
 * ID 0x480 — motor_VibrationLeft  (ADXL345 #1)  @ 100 Hz
 * Parameters:
 *   unsafe : status flag (0 = safe, 1 = unsafe vibration)
 * Payload: 1 byte
 * ---------------------------------------------------------- */
void can_tx_adxl1(uint8_t unsafe);

/* ----------------------------------------------------------
 * ID 0x481 — motor_VibrationRight  (ADXL345 #2)  @ 100 Hz
 * Parameters:
 *   unsafe : status flag (0 = safe, 1 = unsafe vibration)
 * Payload: 1 byte
 * ---------------------------------------------------------- */
void can_tx_adxl2(uint8_t unsafe);

#endif /* CAN_FRAMES_H */
