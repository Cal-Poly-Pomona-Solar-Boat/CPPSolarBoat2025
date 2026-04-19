# Solar Boat Telemetry System

A three-layer real-time telemetry system for a solar boat. Raw physical signals are acquired on an STM32G474RE microcontroller, transmitted over CAN FD to a Raspberry Pi gateway, and fanned out into a live Node-RED dashboard and InfluxDB time-series database.

---

## Repository structure

```
├── STM32/
│   ├── main.c              # Sensor hub — scheduler, ISRs, sensor reads
│   ├── can_frames.c        # CAN FD frame builders and transmit functions
│   └── can_frames.h        # Frame ID map and function declarations
├── Pi/
│   ├── main.c              # C sensor daemon — MCP3008, MPU6050, MQTT publish
│   ├── mcp3008.c           # SPI driver for MCP3008 ADC
│   ├── mpu6050.c           # I2C driver for MPU6050 IMU
│   ├── thermistor.c        # Steinhart-Hart beta model temperature conversion
│   ├── transducer_4_20ma.c # 4-20mA current loop to engineering units
│   └── voltage_divider.c   # Voltage divider ADC to real voltage conversion
├── NodeRED/
│   └── flows.json          # Node-RED flows — CAN decode, dashboard, InfluxDB
└── README.md
```

---

## System overview

```mermaid
graph TD
    subgraph STM32["STM32G474RE - sensor hub"]
        A1[AS5600 x3 - angle sensors]
        A2[ADXL345 x2 - vibration]
        A3[Hall sensors x2 - RPM]
        A4[ADC1 CH7 - throttle]
    end

    subgraph Pi["Raspberry Pi - gateway"]
        B1[socketcan - CAN decoder]
        B2[C daemon - Pi sensors]
    end

    subgraph NR["Node-RED - dashboard and storage"]
        C1[CAN-Routing switch]
        C2[piSensors decoder]
        C3[Dashboard widgets]
        C4[InfluxDB writer]
    end

    A1 & A2 & A3 & A4 -->|CAN FD bus| B1
    B2 -->|MQTT pi/sensors| C2
    B1 --> C1
    C1 --> C3
    C2 --> C3
    C1 --> C4
    C2 --> C4
```

---

## Layer 1 — STM32G474RE sensor hub (`STM32/main.c` + `STM32/can_frames.c`)

The main loop uses a non-blocking `HAL_GetTick()` timestamp pattern. Each sensor has its own `last_tick_*` variable and fires independently when its interval elapses. There are no RTOS tasks or DMA — everything runs cooperatively in a single `while(1)`.

The Hall RPM path is the only genuinely interrupt-driven path. It runs entirely outside the main loop via `EXTI0` and `EXTI1` ISRs so motor speed measurement is never blocked by I2C reads.

### Scheduling model

```mermaid
graph TD
    WL[while 1 - main loop]

    WL -->|every 2ms| S1[Read AS5600 no.1 - I2C3]
    WL -->|every 100ms| S2[Read AS5600 no.2 - I2C4]
    WL -->|every 100ms| S3[Read AS5600 no.3 - I2C2]
    WL -->|every 20ms| S4[Read ADC1 - throttle]
    WL -->|every 1ms| S5[Sample ADXL345 no.1 - I2C3]
    WL -->|every 1ms| S6[Sample ADXL345 no.2 - I2C3]

    S5 -->|10 samples collected| V1[Compute std dev x y z]
    S6 -->|10 samples collected| V2[Compute std dev x y z]
    V1 -->|compare to 0.08g threshold| F1[unsafe boolean]
    V2 -->|compare to 0.08g threshold| F2[unsafe boolean]

    ISR1[EXTI0 ISR - PA0 Hall no.1]
    ISR2[EXTI1 ISR - PA1 Hall no.2]
    TIM2[TIM2 32-bit us counter]

    ISR1 -->|read TIM2 delta| TIM2
    ISR2 -->|read TIM2 delta| TIM2
    TIM2 --> R1[RPM - 5 sample rolling avg]
    TIM2 --> R2[RPM - 5 sample rolling avg]
    R1 -->|3s timeout zeroes reading| CAN
    R2 -->|3s timeout zeroes reading| CAN

    S1 & S2 & S3 & S4 & F1 & F2 --> CAN[CAN FD transmit - FDCAN1]
```

### Sensor interface table

| Sensor | Interface | Signal | Rate |
|---|---|---|---|
| AS5600 #1 | I2C3 · 0x36 | Steering angle | 500 Hz |
| AS5600 #2 | I2C4 · 0x36 | Left rudder position | 10 Hz |
| AS5600 #3 | I2C2 · 0x36 | Right rudder position | 10 Hz |
| ADXL345 #1 | I2C3 · 0xA6 | Motor 1 vibration | 100 Hz (10 samples, 1 ms each) |
| ADXL345 #2 | I2C3 · 0x3A | Motor 2 vibration | 100 Hz (same pattern) |
| Hall sensor #1 | PA0 · EXTI0 | Left motor RPM | ISR-driven (1000 Hz TX) |
| Hall sensor #2 | PA1 · EXTI1 | Right motor RPM | ISR-driven (1000 Hz TX) |
| ADC1 · CH7 | PC1 | Throttle (0–3.3 V) | 50 Hz |

### CAN FD frame map (`STM32/can_frames.c`)

All frames are built and transmitted in `can_frames.c`. Each function receives a sensor value, packs it big-endian into a byte array, and calls `HAL_FDCAN_AddMessageToTxFifoQ`. TX failures print a UART message but do not call `Error_Handler` — the main loop always continues regardless of CAN bus status.

```mermaid
graph LR
    subgraph Sensors
        S1[AS5600 no.1]
        S2[AS5600 no.2]
        S3[AS5600 no.3]
        S4[ADXL345 no.1]
        S5[ADXL345 no.2]
        S6[Hall no.1]
        S7[Hall no.2]
        S8[ADC1 throttle]
    end

    subgraph Frames["CAN FD frames - uint16 BE encoding"]
        F1["0x040 - steering - deg x10 - 500 Hz"]
        F2["0x122 - rudder L - deg x10 - 10 Hz"]
        F3["0x123 - rudder R - deg x10 - 10 Hz"]
        F4["0x480 - vibe L - bool - 100 Hz"]
        F5["0x481 - vibe R - bool - 100 Hz"]
        F6["0x420 - RPM L - rpm x10 - 1000 Hz"]
        F7["0x421 - RPM R - rpm x10 - 1000 Hz"]
        F8["0x2C8 - throttle - V x1000 - 50 Hz"]
    end

    S1 -->|can_tx_steering1| F1
    S2 -->|can_tx_steering2| F2
    S3 -->|can_tx_steering3| F3
    S4 -->|can_tx_adxl1| F4
    S5 -->|can_tx_adxl2| F5
    S6 -->|can_tx_rpm| F6
    S7 -->|can_tx_rpm2| F7
    S8 -->|can_tx_throttle| F8
```

**Encoding examples from `can_frames.h`:**

| Signal | Scaling | Example |
|---|---|---|
| Angle (degrees) | `deg * 10` as `uint16` | 270.5° → 2705 → `[0x0A, 0x91]` |
| RPM | `rpm * 10` as `uint16` | 123.4 RPM → 1234 → `[0x04, 0xD2]` |
| Voltage | `voltage * 1000` as `uint16` | 2.500 V → 2500 → `[0x09, 0xC4]` |
| Vibration | 1-byte boolean | 0 = safe, 1 = unsafe |

### Vibration processing

The ADXL345 sensors accumulate 10 samples into a rolling buffer at 1 ms intervals (controlled by `ADXL_SAMPLE_INTERVAL_MS = RATE_MOTOR_VIBRATION_MS / ADXL_SAMPLES`). On every 10th sample the standard deviation of each axis is computed and compared to `ADXL_THRESH = 0.08g`. The result is a single `uint8_t` boolean transmitted via `can_tx_adxl1` / `can_tx_adxl2`. This deliberately decouples the 1 ms sampling rate from the 10 ms reporting rate — high-frequency events are captured without flooding the CAN bus.

### RPM measurement

Hall sensor edges trigger `HAL_GPIO_EXTI_Callback` on `PA0` and `PA1`. Each ISR reads TIM2's free-running 32-bit counter (170 MHz / prescaler 169 ≈ 1 MHz), computes the pulse-to-pulse period, derives RPM, and maintains a 5-sample rolling average in `rpm_1_buffer` / `rpm_2_buffer`. A 3-second timeout (`RPM_TIMEOUT = 3000000` microseconds) zeroes the reading if no pulse arrives. A minimum pulse gap of 5000 ticks debounces mechanical noise. RPM is then transmitted via `can_tx_rpm` / `can_tx_rpm2`.

---

## Layer 2 — Raspberry Pi gateway (`Pi/`)

Two independent data paths run in parallel.

```mermaid
graph TD
    subgraph PathA["Path A - CAN FD"]
        A1[socketcan-out node - can0]
        A2[CAN-Decoding function node]
        A3[switch on CAN ID]
        A4[JS object - degrees / rpm / voltage / is_unsafe]
    end

    subgraph PathB["Path B - Pi sensors"]
        B1[MCP3008 x3 - SPI - mcp3008.c]
        B2[MPU6050 - I2C - mpu6050.c]
        B3[C daemon main loop - 2 Hz]
        B4[thermistor.c - Steinhart-Hart beta]
        B5[transducer_4_20ma.c - 4 to 20mA]
        B6[voltage_divider.c - divider ratio]
        B7[MQTT publish - pi/sensors JSON]
        B8[mqtt-in Node-RED node]
    end

    A1 --> A2 --> A3 --> A4
    B1 & B2 --> B3
    B3 --> B4 & B5 & B6
    B4 & B5 & B6 --> B7 --> B8
```

**Path A — CAN FD:** The `socketcan-out` Node-RED node reads raw frames from `can0`. The `CAN-Decoding` function node switches on the CAN ID and deserialises each frame into a structured JavaScript object with named fields (`degrees`, `rpm`, `voltage`, `is_unsafe`). The decoded object is broadcast via `link out` nodes to both the dashboard and InfluxDB flows.

**Path B — Pi sensors:** The C daemon (`Pi/main.c`) runs a 2 Hz loop reading three MCP3008 ADCs over SPI (`mcp3008.c`) and an MPU6050 IMU over I2C (`mpu6050.c`). Sensor values are converted using dedicated modules:

| Module | Input | Conversion | Output |
|---|---|---|---|
| `thermistor.c` | ADC counts | Steinhart-Hart β model | Temperature °C |
| `transducer_4_20ma.c` | ADC counts | V / shunt → mA → engineering units | Pressure PSI |
| `voltage_divider.c` | ADC counts | Vnode × (R_top + R_bot) / R_bot | Real voltage V |
| `mpu6050.c` | Raw I2C registers | accel / 16384 · gyro / 131 | g and dps |

All values are serialised into a single JSON string and published to MQTT topic `pi/sensors` via libmosquitto. A `mqtt-in` Node-RED node subscribes and feeds the data downstream.

---

## Layer 3 — Node-RED dashboard and storage (`NodeRED/flows.json`)

```mermaid
graph TD
    CAN[CAN-Routing switch - 8 outputs]
    PI[piSensors decoder - 28 outputs]

    CAN --> G1[Steering gauge - 0x040]
    CAN --> G2[Rudder L gauge - 0x122]
    CAN --> G3[Rudder R gauge - 0x123]
    CAN --> G4[RPM L gauge - 0x420]
    CAN --> G5[RPM R gauge - 0x421]
    CAN --> G6[Vibration L status - 0x480]
    CAN --> G7[Vibration R status - 0x481]
    CAN --> G8[Throttle bar - 0x2C8]

    PI --> D1[Battery temps x6]
    PI --> D2[Panel temps x4]
    PI --> D3[Voltages x7]
    PI --> D4[Boat attitude - roll and pitch]
    PI --> D5[Pressure transducers x3]

    CAN --> RL[PrepCanForInflux - rate limiter]
    RL -->|steering 20Hz - others 10Hz| DB[(InfluxDB v2\ncan_sensor_data)]
    PI -->|output 28| DB2[(InfluxDB v2\nPI_sensor_data)]
```

**Dashboard routing.** The `CAN-Routing` switch node inspects `msg.payload.signal` and fans the stream to eight dedicated UI widgets — half-gauges for the three angle signals and both RPM signals, Vue template components for the two vibration status indicators (green/red), and a vertical bar template for throttle voltage.

**Pi sensor decoding.** The `piSensors` function node parses the JSON payload published by the C daemon and emits 28 separate output messages wired to `ui-text` nodes for temperatures, pressures, voltages, and IMU values. Output 28 is a pre-formatted Stackhero InfluxDB write message.

**Notable dashboard components:**
- Battery bar — SVG `clip-path` scaled by `msg.payload` percentage, tri-colour fill (red → amber → green)
- Boat attitude indicator — artificial horizon driven by real IMU roll/pitch from `mpu6050.c`, SVG `rotate()` and `translate` applied reactively via Vue watchers

### InfluxDB write rate limiting

```mermaid
graph TD
    RPM[RPM frames - 1000 Hz raw]
    ST[Steering frames - 500 Hz raw]
    OT[All other frames - 10 to 100 Hz raw]

    RPM -->|rate limited| W1[10 Hz written]
    ST -->|rate limited| W2[20 Hz written]
    OT -->|rate limited| W3[10 Hz written]

    W1 -->|864000 points per day| DB
    W2 -->|1728000 points per day| DB
    W3 -->|864000 points per day| DB

    DB[(InfluxDB v2\nSolarBoatData2025-2026)]

    NOTE["Without limiter: RPM alone = 86 million points per day"]
```

The `PrepCanForInflux` function node applies per-signal rate limiting using node context timestamps. Pi sensor data is written under a separate measurement `PI_sensor_data` tagged with `device: raspberry_pi` and `source: c_program`.

---

## Key design decisions

| Decision | Reason |
|---|---|
| Non-blocking `HAL_GetTick()` scheduler | No RTOS needed — sensors fire independently without blocking each other |
| Hall RPM via EXTI ISR not main loop | Guarantees microsecond-accurate pulse timing regardless of I2C bus load |
| ADXL 10-sample buffer before reporting | Decouples 1 ms sampling from 10 ms CAN TX — captures events without flooding the bus |
| `uint16_t` ×10 / ×1000 fixed-point encoding in `can_frames.c` | Avoids floating point in CAN payloads while preserving one decimal place of precision |
| CAN TX failures print not Error_Handler | Main loop always continues — a busy CAN bus does not halt the sensor hub |
| Separate modules for each Pi sensor type | `thermistor.c`, `transducer_4_20ma.c`, `voltage_divider.c` are independently testable |
| MQTT for Pi sensor data | Decouples the C daemon from Node-RED — either can restart independently |
| InfluxDB write-rate limiter in node context | Reduces 86M points/day to ~864K points/day for RPM without losing dashboard resolution |
