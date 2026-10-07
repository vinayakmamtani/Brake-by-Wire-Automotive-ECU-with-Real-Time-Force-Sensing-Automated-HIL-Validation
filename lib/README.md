# Electro-Mechanical Brake-by-Wire (BBW) ECU & Automated HIL Validation Suite

![ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)
![RTOS](https://img.shields.io/badge/OS-FreeRTOS-green.svg)
![Language](https://img.shields.io/badge/Language-C%20%2F%20Python-orange.svg)
![Validation](https://img.shields.io/badge/Testing-Automated%20HIL-red.svg)

A proof-of-concept dry Brake-by-Wire (BBW) pedal-transducer ECU built on an ESP32 dual-core architecture running FreeRTOS. The system decouples driver pedal force from physical hydraulic lines, executing deterministic 50 Hz pedal acquisition, digital low-pass filtering, and ISO 26262-inspired fault containment, verified via an automated external Python Hardware-in-the-Loop (HIL) test harness.

---

## System Architecture

The firmware utilizes **Asymmetric Multiprocessing (AMP)** on the ESP32 to eliminate UI jitter from the safety-critical brake calculation loop.

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 ESP32 Dual-Core ECU                    │
                  │                                                        │
[HX711 24-bit] ──>│  Core 1 (Priority 5, 50 Hz):                           │
 [Load Cell]      │  • 24-bit Sign Extension                               │
                  │  • Boot-time Tare Offset Gating                        │
                  │  • EMA Low-Pass Filter (alpha = 0.25)                  │
                  │  • Rail Saturation Check (|RAW| > 8M)                  │
                  │  • Mutex Write Lock                                    │
                  │           │                                            │
                  │           ▼                                            │
                  │    [g_telemetry_mutex] ◄── Priority Inheritance Enforced
                  │           ▲                                            │
                  │           │                                            │
                  │  Core 0 (Priority 1, 10 Hz):                           │
                  │  • Mutex Read Snapshot                                 │
                  │  • SSD1306 Page Buffering                              │
                  │  • I2C Display Flush (400 kHz) ──> [SSD1306 OLED]      │
                  └─────────────────────────┬──────────────────────────────┘
                                            │ UART (115200 Baud)
                                            ▼
                           ┌─────────────────────────────────┐
                           │   Python Automated HIL Runner   │
                           │   • TC-01: Null Stability       │
                           │   • TC-02: Transfer Monotonicity│
                           │   • TC-03: Fault-Injection Safe │
                           └─────────────────────────────────┘
```

### FreeRTOS Task Scheduling & Concurrency

| Task Name | Target Core | Priority | Frequency | Execution Role |
| :--- | :---: | :---: | :---: | :--- |
| `bbw_control_task` | **Core 1** | **5** (High) | **50 Hz** (20 ms) | Sensor readout, EMA signal filtering, failsafe gating, and safe-state clamping. |
| `display_task` | **Core 0** | **1** (Low) | **10 Hz** (100 ms) | Asynchronous OLED gauge rendering over blocking I2C. Isolated from Core 1. |

> **Concurrency Design:** A FreeRTOS Mutex (`xSemaphoreCreateMutex`) guards the shared `bbw_telemetry_t` memory structure. A mutex was selected over a binary semaphore to enforce **Priority Inheritance**, mitigating **Unbounded Priority Inversion** where intermediate OS tasks could otherwise block the 50 Hz brake control loop.

---

## Signal Conditioning & Safety Pipeline

1. **24-Bit Two's Complement Sign Extension:** Manual sign padding (`data |= 0xFF000000` when bit 23 is set) prevents negative counts from wrapping to positive saturation (+16.7M counts).
2. **Dynamic Zeroing (Tare):** Boot-sequence averages 20 samples to establish zero-force offset and compensate for mechanical preload.
3. **Deadband Threshold (15,000 counts):** Suppresses mechanical cabin vibration and thermal ADC noise to prevent inadvertent **Phantom Braking**.
4. **Exponential Moving Average (EMA) Filter:** Digital 1st-order IIR filter (alpha = 0.25) eliminates high-frequency electromagnetic noise while maintaining sub-40 ms phase lag:
   `y[k] = alpha * x[k] + (1 - alpha) * y[k-1]`
5. **Open-Circuit Failsafe (Safe Torque Off):** Detects ADC rail saturation (`|RAW| > 8,000,000`) indicative of broken strain gauge wiring or short circuits, forcing the brake command to **0% Safe State** within one cycle (< 20 ms).

---

## Automated Hardware-in-the-Loop (HIL) Validation

Validation is handled by an automated external test harness (`tests/test/hil_test_runner.py`) interfacing over UART at 115200 baud. Telemetry frames are parsed in real time using structured regex assertions.

```
============================================================
              AUTOMATED HIL TEST BENCH REPORT               
============================================================
TARGET: ESP32 Dual-Core Brake-by-Wire Controller
COMMUNICATION: UART @ 115200 Baud

[RUNNING] TC-01: Null-Load Zero Stability (50 Samples)...
  --> Checking: Null drift <= 0% across 50 consecutive frames
  --> PASS: Zero output asserted. Deadband successfully suppressed drift.

[RUNNING] TC-02: Dynamic Transfer Function Verification (6.0s Deflection)...
  --> Checking: System achieves active threshold (>= 70%) without saturation
  --> PASS: Monotonic force tracking verified. Peak demand: 82%.

[RUNNING] TC-03: Sensor Open-Circuit Plausibility Assertion...
  --> PASS: Rail fault tripped. Telemetry verified in [FAULTSAFE: 0%].

SUMMARY: 3 / 3 TEST CASES PASSED (0 REGRESSIONS)
============================================================
```

---

## Repository Structure

```text
├── docs/
│   └── architecture_diagram.png    # Core allocation and signal routing
├── firmware/
│   ├── src/
│   │   └── main.c                  # Dual-core FreeRTOS firmware implementation
│   └── platformio.ini              # Build configuration and pin environments
├── tests/
│   └── test/
│       ├── hil_test_runner.py      # Automated Python HIL verification suite
│       └── hil_report.txt          # Captured test bench execution logs
└── README.md
```

---

## Hardware Configuration

* **Microcontroller:** ESP32-WROOM-32 (Xtensa Dual-Core 32-bit LX6 @ 240 MHz)
* **Sensor:** Micro Cantilever Load Cell (Strain Gauge Bridge)
* **ADC:** HX711 24-bit ADC (Ch A, Gain 128)
  * `DT`: GPIO 19
  * `SCK`: GPIO 18
* **Telemetry Display:** SSD1306 0.96" OLED (I2C @ 400 kHz)
  * `SDA`: GPIO 21
  * `SCL`: GPIO 22

---

## Getting Started

### 1. Build & Flash Firmware
Using PlatformIO CLI:
```bash
pio run --target upload
```

### 2. Run Automated HIL Bench Tests
Ensure the ESP32 is connected via USB and execute the test runner:
```bash
python tests/test/hil_test_runner.py --port COM3
```

---

## Future Enhancements
* CAN-FD / TWAI (ISO 11898-1) communication interface to broadcast brake requests to electronic brake calipers.
* Dual-channel redundant strain gauge architecture for ASIL-D fault tolerance.
* Dynamic pedal map switching (Linear Sport vs. Progressive Comfort curves).
