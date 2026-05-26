# Solar Boat Telemetry System

A three-layer real-time telemetry system for the CPP Solar Boat. Raw physical signals are acquired on an STM32G474RE microcontroller, transmitted over CAN FD to a Raspberry Pi gateway, and fanned out into a live Node-RED dashboard and InfluxDB time-series database.

---

## Repository structure

```text
├── STM32/
│   ├── main.c              # Sensor hub — cooperative scheduler, ISRs, sensor reads
│   ├── can_frames.c        # CAN FD frame builders and transmit functions
│   └── can_frames.h        # Frame ID constants and function declarations
├── Pi/
│   ├── main.c              # C sensor daemon — 2 Hz loop, MCP3008 + MPU6050, MQTT publish
│   ├── mcp3008.c/.h        # SPI driver for MCP3008 10-bit ADC (returns 0–1023)
│   ├── mpu6050.c/.h        # I2C driver for MPU6050 IMU (accel g, gyro °/s, temp °C)
│   ├── thermistor.c/.h     # Steinhart–Hart β model: ADC counts → temperature °C
│   ├── transducer_4_20ma.c/.h  # 4–20 mA current loop: ADC counts → engineering units (e.g. PSI)
│   └── voltage_divider.c/.h    # Resistor divider inversion: ADC counts → real voltage V
├── NodeRED/
│   └── flows.json          # Node-RED flows — CAN decode, dashboard widgets, InfluxDB writes
└── README.md
```

---

## System overview

```
STM32G474RE  ──CAN FD bus──▶  Raspberry Pi  ──▶  Node-RED dashboard (browser)
(sensor hub)                  (gateway)      └──▶  InfluxDB v2 (time-series log)
                                             
                Pi also runs its own C sensor daemon, publishing
                local ADC + IMU readings to an MQTT broker.
```

**CAN FD** (Controller Area Network with Flexible Data-rate) is a differential two-wire serial bus originally designed for automotive use. It is noise-tolerant, supports up to 64-byte payloads, and allows multiple nodes to share a single bus. Each message carries a numeric ID (e.g. `0x040`) that identifies the signal type.

**MQTT** (Message Queuing Telemetry Transport) is a lightweight publish/subscribe protocol. The C daemon *publishes* to a topic; Node-RED *subscribes* and receives data automatically. The two processes are fully decoupled — either can restart independently.

---

## Layer 1 — STM32G474RE sensor hub (`STM32/`)

The STM32G474RE is a 32-bit ARM Cortex-M4 microcontroller. It runs bare-metal C (no operating system), directly interfacing with sensors over I2C, SPI, and GPIO pins. Its job is to read every sensor on a tight schedule and pack the results into CAN FD frames.

> **I2C** (Inter-Integrated Circuit) — a two-wire bus (clock + data) for communicating with sensors on the same PCB. Each device has a unique 7-bit address, e.g. `0x36`.  
> **GPIO** (General-Purpose Input/Output) — a configurable digital pin. `PA0` and `PA1` are configured as interrupt inputs that trigger on voltage edges from the Hall sensors.  
> **ADC** (Analog-to-Digital Converter) — hardware that samples a continuously-varying voltage and converts it to a digital integer. Used here to read the throttle potentiometer.

### Scheduling model

Rather than using an RTOS (Real-Time Operating System — a small OS that manages multiple concurrent tasks), the firmware uses a **non-blocking cooperative scheduler**: a single `while(1)` loop that calls `HAL_GetTick()` on every iteration to get the current millisecond timestamp. Each sensor has its own `last_tick_*` variable and fires independently when its configured interval elapses. Because no sensor ever blocks inside the loop, all sensors stay on schedule regardless of each other.

**The one exception is RPM**, which requires microsecond-accurate pulse timing that the main loop cannot guarantee — it might be mid-way through an I2C transaction when a Hall edge arrives. RPM instead uses hardware interrupts:

**ISR (Interrupt Service Routine)** — a special function that the hardware calls automatically, suspending whatever the CPU was doing, the instant a configured event occurs. Here, rising and falling edges on `PA0`/`PA1` trigger `HAL_GPIO_EXTI_Callback`. The ISR reads TIM2's free-running 32-bit hardware timer (prescaled to ~1 MHz, giving 1 µs resolution), computes the pulse period, and returns. The main loop never knows it was interrupted.

#### Main loop path

```
while(1) — HAL_GetTick() scheduler
  ├─ every   2 ms  →  AS5600 #1 steering angle    (I2C3, addr 0x36)
  ├─ every 100 ms  →  AS5600 #2 left rudder angle  (I2C4, addr 0x36)
  ├─ every 100 ms  →  AS5600 #3 right rudder angle (I2C2, addr 0x36)
  ├─ every  20 ms  →  ADC1 CH7 throttle voltage    (PC1, 0–3.3 V)
  ├─ every   1 ms  →  ADXL345 ×2 vibration sample  (I2C3, addrs 0xA6 / 0x3A)
  │                     accumulate 10 samples into ring buffer
  │                     on 10th sample: compute std deviation per axis
  │                     compare to 0.08 g threshold → unsafe boolean
  └─ all signals   →  pack into CAN FD frames → transmit via FDCAN1
```

#### ISR path (runs outside the main loop, hardware-triggered)

```
PA0 rising/falling edge  →  EXTI0 ISR  ─┐
PA1 rising/falling edge  →  EXTI1 ISR  ─┤
                                         ├─ read TIM2 counter (1 MHz, 32-bit)
                                         ├─ compute pulse period → RPM
                                         ├─ push into 5-sample rolling average
                                         ├─ if no pulse for 3 s → output 0
                                         ├─ debounce: ignore gaps < 5 000 µs
                                         └─ transmit via FDCAN1 at 1000 Hz
```

### Sensor interface table

| Sensor | Interface | Address / Pin | Signal | Rate |
|---|---|---|---|---|
| AS5600 #1 | I2C3 | 0x36 | Steering angle | 500 Hz |
| AS5600 #2 | I2C4 | 0x36 | Left rudder position | 10 Hz |
| AS5600 #3 | I2C2 | 0x36 | Right rudder position | 10 Hz |
| ADXL345 #1 | I2C3 | 0xA6 | Motor 1 vibration | 100 Hz (10 samples × 1 ms) |
| ADXL345 #2 | I2C3 | 0x3A | Motor 2 vibration | 100 Hz (10 samples × 1 ms) |
| Hall sensor #1 | GPIO | PA0 · EXTI0 | Left motor RPM | ISR-driven, 1000 Hz TX |
| Hall sensor #2 | GPIO | PA1 · EXTI1 | Right motor RPM | ISR-driven, 1000 Hz TX |
| ADC1 CH7 | Analog | PC1 | Throttle (0–3.3 V) | 50 Hz |

### CAN FD frame map (`STM32/can_frames.c`)

Each `can_tx_*` function in `can_frames.c` accepts a sensor value, scales it to a fixed-point integer, packs it big-endian (most-significant byte first) into a byte array, and calls `HAL_FDCAN_AddMessageToTxFifoQ` to enqueue the frame for hardware transmission. TX failures log a UART debug message but do **not** halt execution — a temporarily busy bus does not stall sensor reads.

**Why fixed-point encoding?** CAN payloads are raw bytes with no floating-point standard. Values are scaled before transmission: e.g. 270.5° × 10 = 2705 → `[0x0A, 0x91]`. The receiver divides by the scale factor to recover the original value.

| Frame ID | Signal | Encoding | Rate |
|---|---|---|---|
| `0x040` | Steering angle | `uint16` big-endian, deg × 10 | 500 Hz |
| `0x122` | Left rudder angle | `uint16` big-endian, deg × 10 | 10 Hz |
| `0x123` | Right rudder angle | `uint16` big-endian, deg × 10 | 10 Hz |
| `0x480` | Vibration L (motor 1) | `uint8` boolean — 0 safe, 1 unsafe | 100 Hz |
| `0x481` | Vibration R (motor 2) | `uint8` boolean — 0 safe, 1 unsafe | 100 Hz |
| `0x420` | RPM left | `uint16` big-endian, rpm × 10 | 1000 Hz |
| `0x421` | RPM right | `uint16` big-endian, rpm × 10 | 1000 Hz |
| `0x2C8` | Throttle voltage | `uint16` big-endian, V × 1000 | 50 Hz |

**Encoding examples:**

| Signal | Raw value | Scaled integer | Bytes on wire |
|---|---|---|---|
| Angle | 270.5° | 2705 | `[0x0A, 0x91]` |
| RPM | 123.4 rpm | 1234 | `[0x04, 0xD2]` |
| Voltage | 2.500 V | 2500 | `[0x09, 0xC4]` |
| Vibration | unsafe | 1 | `[0x01]` |

### Vibration processing

The ADXL345 is a 3-axis MEMS accelerometer. Rather than transmitting raw samples (which would saturate the CAN bus at 3 × 100 Hz × 2 sensors), the firmware uses a two-rate design:

1. **Sample rate (1 ms):** Accumulate one acceleration reading per axis per millisecond into a 10-element ring buffer, controlled by `ADXL_SAMPLE_INTERVAL_MS = RATE_MOTOR_VIBRATION_MS / ADXL_SAMPLES`.
2. **Report rate (10 ms):** On every 10th sample, compute the standard deviation of each axis across the buffer and compare to `ADXL_THRESH = 0.08 g`. The result is a single `uint8_t` boolean: 0 = all axes within threshold (safe), 1 = at least one axis exceeded it (unsafe).

This captures short, high-energy vibration events without flooding the bus.

### RPM measurement

Each Hall-effect sensor produces a square wave whose period is inversely proportional to motor speed. Inside `HAL_GPIO_EXTI_Callback`:

1. Read TIM2 (170 MHz system clock ÷ prescaler 169 ≈ 1 MHz → 1 µs per tick).
2. Subtract the previous timestamp to get the pulse period in microseconds.
3. Derive RPM: `rpm = 60_000_000 / period_us` (adjust for poles as needed).
4. Push into a 5-element circular buffer; transmit the rolling average (reduces single-edge noise).
5. Timeout: if no edge arrives within `RPM_TIMEOUT = 3 000 000 µs` (3 s), output RPM = 0 to prevent stale readings from persisting.
6. Debounce: ignore any edge arriving less than 5 000 ticks (5 ms) after the previous one, to reject mechanical contact bounce.

---

## Layer 2 — Raspberry Pi gateway (`Pi/`)

The Raspberry Pi runs Linux and handles two independent, parallel data paths.

### Path A — CAN FD decoding (Node-RED)

**SocketCAN** is a Linux kernel subsystem that exposes CAN bus adapters as standard network interfaces (just like `eth0` for Ethernet). This means userspace programs — including Node-RED — can read and write CAN frames using ordinary socket APIs without any custom kernel code.

The `socketcan-out` Node-RED node reads raw frames from the `can0` interface. The `CAN-Decoding` function node switches on the frame ID and deserialises each frame into a structured JavaScript object with named fields:

```
{
  signal: "steering",   // human-readable name
  degrees: 270.5,       // engineering value (bytes / scale factor)
  raw: [0x0A, 0x91]     // original bytes for diagnostics
}
```

Decoded objects are broadcast via `link out` nodes to both the dashboard and InfluxDB flows.

### Path B — Pi sensor daemon (`Pi/main.c`)

A **daemon** is a background process that the OS starts at boot and keeps running continuously, with no controlling terminal. `Pi/main.c` compiles to a standalone binary that loops at 2 Hz, reading local sensors and publishing results to MQTT.

**SPI** (Serial Peripheral Interface) — a four-wire synchronous bus (clock, chip-select, MOSI, MISO). Faster than I2C and suitable for the MCP3008 ADC chips.

#### MCP3008 ADC (`mcp3008.c`)

```c
int  mcp3008_open(const char *spidev, unsigned mode, unsigned speed_hz);
void mcp3008_close(int fd);
int  mcp3008_read(int fd, int channel);  // returns 0–1023, or -1 on error
```

The MCP3008 is an 8-channel, 10-bit SPI ADC. `mcp3008_read` performs a single-ended conversion on the selected channel and returns a raw count from 0 to 1023, representing 0 V to Vref. Three MCP3008 chips provide 24 ADC channels total for thermistors, pressure transducers, and voltage dividers.

#### MPU6050 IMU (`mpu6050.c`)

```c
typedef struct {
    double ax_g, ay_g, az_g;     // linear acceleration in g (1 g ≈ 9.81 m/s²)
    double gx_dps, gy_dps, gz_dps; // angular rate in degrees per second
    double temp_c;               // on-die temperature sensor
} mpu6050_sample_t;

int  mpu6050_open(const char *i2cdev);
void mpu6050_close(int fd);
int  mpu6050_init(int fd);
int  mpu6050_read_sample(int fd, mpu6050_sample_t *out);
```

The MPU6050 is a 6-axis MEMS IMU (Inertial Measurement Unit) with a 3-axis accelerometer and a 3-axis gyroscope, read over I2C. Raw 16-bit register values are converted to engineering units by `mpu6050_read_sample`: accelerometer counts are divided by 16 384 LSB/g, gyroscope counts by 131 LSB/°/s. Roll and pitch are derived from the accelerometer axes and used to drive the dashboard attitude indicator.

#### Sensor conversion modules

Each module is independently compilable and testable.

**`thermistor.c`**

```c
typedef struct {
    double vref;           // ADC reference voltage (e.g. 3.3 V)
    double r_fixed_ohms;   // top resistor in voltage divider
    double r0_ohms;        // thermistor nominal resistance at t0 (e.g. 10 kΩ at 25 °C)
    double t0_c;           // reference temperature, typically 25 °C
    double beta;           // material constant (typically 3000–5000 K)
} thermistor_cfg_t;

double thermistor_c_from_adc(int adc_counts, const thermistor_cfg_t *cfg);
```

Converts a raw ADC count to temperature in °C using the **Steinhart–Hart β approximation**:

```
V_node  = adc_counts / 1023.0 × vref
R_therm = r_fixed × V_node / (vref − V_node)
1/T     = 1/T0 + (1/β) × ln(R_therm / R0)   [T in Kelvin]
```

Used for 6 battery pack temperature sensors and 4 solar panel temperature sensors.

**`transducer_4_20ma.c`**

```c
typedef struct {
    double vref;
    double shunt_ohms;  // resistor that converts current to a measurable voltage (V = I × R)
    double ma_min;      // current at zero output, typically 4.0 mA
    double ma_max;      // current at full-scale output, typically 20.0 mA
    double eng_min;     // engineering value at 4 mA (e.g. 0 PSI)
    double eng_max;     // engineering value at 20 mA (e.g. 100 PSI)
} transducer_cfg_t;

double transducer_from_adc(int adc_counts, const transducer_cfg_t *cfg);
double transducer_eng_from_ma(double ma, const transducer_cfg_t *cfg);
```

4–20 mA current-loop transducers transmit their measurement as a current rather than a voltage, making them immune to wiring resistance and noise over long cable runs. The shunt resistor converts the loop current to a voltage that the MCP3008 can measure. Conversion steps:

```
V_shunt = adc_counts / 1023.0 × vref
I_ma    = (V_shunt / shunt_ohms) × 1000
eng     = eng_min + (I_ma − ma_min) / (ma_max − ma_min) × (eng_max − eng_min)
```

Used for 3 coolant/hydraulic pressure transducers.

**`voltage_divider.c`**

```c
typedef struct {
    double vref;
    double r_top_ohms;
    double r_bot_ohms;
} voltage_divider_cfg_t;

double voltage_from_adc(int adc_counts, const voltage_divider_cfg_t *cfg);
```

High voltages (e.g. 48 V battery bus) must be scaled down to the ADC's 0–3.3 V input range using a resistor divider. This function inverts that scaling to recover the real voltage:

```
V_adc  = adc_counts / 1023.0 × vref
V_real = V_adc × (r_top + r_bot) / r_bot
```

Used for 7 voltage rails across the battery management system and power distribution board.

#### MQTT publish

All converted values are serialised into a single JSON string and published to topic `pi/sensors` via **libmosquitto** (the C client library for the Mosquitto MQTT broker). A `mqtt-in` Node-RED node subscribes to this topic and feeds data into the dashboard and InfluxDB flows. Because MQTT is the interface between the daemon and Node-RED, either side can crash and restart without the other losing state.

---

## Layer 3 — Node-RED dashboard and storage (`NodeRED/flows.json`)

**Node-RED** is a browser-based visual programming tool. Logic is expressed as *flows* — directed graphs of *nodes* wired together. Data passes between nodes as `msg` JavaScript objects. The entire flow configuration is stored in `flows.json` and loaded at startup.

### CAN signal routing

The `CAN-Routing` switch node inspects `msg.payload.signal` and routes each decoded frame to its dedicated UI widget:

| Widget type | Signals |
|---|---|
| Half-gauge (arc gauge, 0–360°) | Steering, Rudder L, Rudder R |
| Half-gauge (RPM range) | RPM L, RPM R |
| Vue template — green/red indicator | Vibration L, Vibration R |
| Vertical bar template | Throttle voltage |

### Pi sensor decoding

The `piSensors` function node parses the JSON payload from the MQTT topic and emits **28 separate output messages**, each wired to a `ui-text` display node or a template widget. Output 28 is a pre-formatted Stackhero InfluxDB write message.

| Output group | Count | Source |
|---|---|---|
| Battery pack temperatures | 6 | `thermistor_c_from_adc` |
| Solar panel temperatures | 4 | `thermistor_c_from_adc` |
| Voltage rails | 7 | `voltage_from_adc` |
| Pressure transducers | 3 | `transducer_from_adc` |
| Boat roll and pitch | 2 | `mpu6050_read_sample` |
| InfluxDB write message | 1 | pre-formatted payload |

### Notable dashboard components

**Battery bar** — an SVG element using a `clip-path` rectangle scaled by `msg.payload` percentage. The fill colour transitions through three bands: red (< 20 %), amber (20–50 %), green (> 50 %), implemented as layered SVG fills with clip boundaries driven by the payload value.

**Boat attitude indicator** — an artificial horizon SVG driven by live roll and pitch from the MPU6050. Vue.js watchers on `msg.payload.roll` and `msg.payload.pitch` apply `rotate()` and `translate` transforms to the horizon line and aircraft symbol reactively on every incoming MQTT message.

### InfluxDB write-rate limiting

Raw sensor data arrives far faster than is useful to store:

| Signal | Raw TX rate | Limited write rate | Est. points / day |
|---|---|---|---|
| RPM | 1000 Hz | 10 Hz | ~864 000 |
| Steering | 500 Hz | 20 Hz | ~1 728 000 |
| Other CAN signals | 10–100 Hz | 10 Hz | ~864 000 each |
| Pi sensors | 2 Hz | 2 Hz (no limit needed) | ~172 800 |
| RPM without limiter | 1000 Hz | — | ~86 400 000 |

The `PrepCanForInflux` function node applies per-signal rate limiting using **node context** — persistent in-memory state stored inside a Node-RED node that survives between messages but resets on restart. Each signal stores its last-written timestamp; a new InfluxDB write is only issued when the configured minimum interval has elapsed.

Pi sensor data is written to the `PI_sensor_data` measurement, tagged with `device: raspberry_pi` and `source: c_program`, keeping it separate from CAN-sourced data for cleaner queries.

The InfluxDB bucket is `SolarBoatData2025-2026` on the Stackhero-hosted InfluxDB v2 instance.

---

## Key design decisions

| Decision | Reason |
|---|---|
| Non-blocking `HAL_GetTick()` scheduler instead of RTOS | Simpler than RTOS task management; sensors fire independently without blocking each other, and there is no context-switching overhead |
| Hall RPM via EXTI ISR rather than polled in the main loop | The main loop can be stalled by I2C transactions; the ISR fires the instant the edge arrives, guaranteeing microsecond-accurate pulse capture regardless of I2C bus activity |
| ADXL 10-sample buffer before reporting | Decouples the 1 ms sample rate from the 10 ms CAN TX rate — short vibration bursts are captured without flooding the bus |
| `uint16_t` ×10 / ×1000 fixed-point encoding in CAN frames | Avoids floating-point in CAN payloads (no standard representation in raw bytes) while preserving one decimal place of precision |
| CAN TX failures print to UART, not `Error_Handler` | A busy or disconnected CAN bus does not halt sensor acquisition or lock up the microcontroller |
| Separate `.c/.h` modules for each Pi sensor type | `thermistor.c`, `transducer_4_20ma.c`, `voltage_divider.c` are independently compilable and unit-testable with mock ADC inputs |
| MQTT for Pi sensor data | Fully decouples the C daemon from Node-RED — either process can crash and restart without the other losing state or blocking |
| InfluxDB write-rate limiter using node context | Reduces RPM storage from ~86 M to ~864 K points/day while maintaining full dashboard resolution — Node-RED reads the high-rate stream, InfluxDB only stores the decimated version |
