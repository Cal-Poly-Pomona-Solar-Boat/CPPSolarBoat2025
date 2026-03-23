// ============================================================
// CAN FD Frame Decoder — STM32 Sensor Hub
// Runs on every incoming message from the socketcan-in node
// ============================================================

msg.result = {};

// msg.payload.canid  — the CAN frame ID as a number (e.g. 0, 1, 2...)
// msg.payload.data   — array of bytes received
const id   = msg.payload.canid;
const data = msg.payload.data;

switch (id) {

    // ----------------------------------------------------------
    // Frame 0x000 — AS5600 #1 steering angle (I2C3)
    // STM32 packs: deg * 10 as uint16, big-endian, 2 bytes
    // Example: 359.9 deg → 3599 → [0x0E, 0x0F]
    // ----------------------------------------------------------
    case 0:
        let deg_int_1 = (data[0] << 8) | data[1];
        let degrees_1 = deg_int_1 / 10.0;
        msg.result = {
            frame:   "0x000",
            sensor:  "AS5600_1",
            degrees: degrees_1,
            raw: [data[0].toString(16).toUpperCase(),
                  data[1].toString(16).toUpperCase()]
        };
        break;

    // ----------------------------------------------------------
    // Frame 0x001 — dummy frame (proof of concept)
    // STM32 packs: 0xDE 0xAD hardcoded
    // ----------------------------------------------------------
    case 1:
        msg.result = {
            frame:  "0x001",
            sensor: "dummy",
            raw: [data[0].toString(16).toUpperCase(),
                  data[1].toString(16).toUpperCase()]
        };
        break;

    // ----------------------------------------------------------
    // Frame 0x002 — ADXL345 #1 motor 1 vibration
    // STM32 packs: each axis * 100 as int16, big-endian, 8 bytes
    // Bytes 0-1: X, 2-3: Y, 4-5: Z, 6-7: padding zeros
    // Example: -0.98g → -98 → [0xFF, 0x9E]
    //
    // int16 sign extension: if raw > 32767, subtract 65536
    // This converts the unsigned JS number back to signed
    // e.g. 0xFF9E = 65438 → 65438 - 65536 = -98 → -0.98g
    // ----------------------------------------------------------
    case 2:
        let x1_raw = (data[0] << 8) | data[1];
        let y1_raw = (data[2] << 8) | data[3];
        let z1_raw = (data[4] << 8) | data[5];
        if (x1_raw > 32767) x1_raw -= 65536;
        if (y1_raw > 32767) y1_raw -= 65536;
        if (z1_raw > 32767) z1_raw -= 65536;
        msg.result = {
            frame:  "0x002",
            sensor: "ADXL345_1",
            x_g:    x1_raw / 100.0,
            y_g:    y1_raw / 100.0,
            z_g:    z1_raw / 100.0
        };
        break;

    // ----------------------------------------------------------
    // Frame 0x003 — ADXL345 #2 motor 2 vibration
    // Same encoding as 0x002
    // ----------------------------------------------------------
    case 3:
        let x2_raw = (data[0] << 8) | data[1];
        let y2_raw = (data[2] << 8) | data[3];
        let z2_raw = (data[4] << 8) | data[5];
        if (x2_raw > 32767) x2_raw -= 65536;
        if (y2_raw > 32767) y2_raw -= 65536;
        if (z2_raw > 32767) z2_raw -= 65536;
        msg.result = {
            frame:  "0x003",
            sensor: "ADXL345_2",
            x_g:    x2_raw / 100.0,
            y_g:    y2_raw / 100.0,
            z_g:    z2_raw / 100.0
        };
        break;

    // ----------------------------------------------------------
    // Frame 0x004 — Hall sensor RPM
    // STM32 packs: rpm * 10 as uint16, big-endian, 2 bytes
    // Example: 123.4 RPM → 1234 → [0x04, 0xD2]
    // ----------------------------------------------------------
    case 4:
        let rpm_int = (data[0] << 8) | data[1];
        let rpm     = rpm_int / 10.0;
        msg.result = {
            frame:  "0x004",
            sensor: "hall_rpm",
            rpm:    rpm,
            raw: [data[0].toString(16).toUpperCase(),
                  data[1].toString(16).toUpperCase()]
        };
        break;

    // ----------------------------------------------------------
    // Frame 0x005 — Quadrature encoder
    // STM32 packs: counts as int16 + angle*10 as int16, 4 bytes
    // Bytes 0-1: counts (signed), 2-3: angle*10 (signed)
    //
    // Both values need int16 sign extension (same as ADXL above)
    // Example: counts=-512 → [0xFE, 0x00]
    //          angle=-45.2 → -452 → [0xFE, 0x3C]
    // ----------------------------------------------------------
    case 5:
        let counts_raw = (data[0] << 8) | data[1];
        let angle_raw  = (data[2] << 8) | data[3];
        if (counts_raw > 32767) counts_raw -= 65536;
        if (angle_raw  > 32767) angle_raw  -= 65536;
        msg.result = {
            frame:     "0x005",
            sensor:    "encoder",
            counts:    counts_raw,
            angle_deg: angle_raw / 10.0
        };
        break;

    // ----------------------------------------------------------
    // Frame 0x006 — AS5600 #2 steering angle (I2C4)
    // Same encoding as 0x000
    // ----------------------------------------------------------
    case 6:
        let deg_int_2 = (data[0] << 8) | data[1];
        let degrees_2 = deg_int_2 / 10.0;
        msg.result = {
            frame:   "0x006",
            sensor:  "AS5600_2",
            degrees: degrees_2,
            raw: [data[0].toString(16).toUpperCase(),
                  data[1].toString(16).toUpperCase()]
        };
        break;

    // ----------------------------------------------------------
    // Catch-all for any unrecognised frame ID
    // ----------------------------------------------------------
    default:
        msg.result = {
            frame:  id,
            sensor: "unknown"
        };
        break;
}

msg.payload = msg.result;
return msg;
