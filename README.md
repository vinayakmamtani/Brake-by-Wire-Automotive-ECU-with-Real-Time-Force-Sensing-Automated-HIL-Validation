# Electro-Mechanical Brake-by-Wire (BBW) ECU & Automated HIL Validation Suite

![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg?logo=espressif)
![Framework](https://img.shields.io/badge/Framework-ESP--IDF%20%2F%20FreeRTOS-green.svg)
![Language](https://img.shields.io/badge/Language-C%20%2F%20Python-orange.svg)
![Validation](https://img.shields.io/badge/Testing-Automated%20HIL-red.svg)
![Standard](https://img.shields.io/badge/Design-ISO%2026262%20Principles-purple.svg)
![License](https://img.shields.io/badge/License-MIT-brightgreen.svg)

A proof-of-concept **Dry Brake-by-Wire (BBW) Pedal-Transducer Electronic Control Unit (ECU)** developed on an ESP32 dual-core Xtensa LX6 architecture running FreeRTOS. The system decouples physical hydraulic actuation from driver pedal input, executing a deterministic **50 Hz** force-sensing loop, 1st-order digital IIR filtering, and ISO 26262-inspired fault containment with safe-state clamping. Verification is driven by an automated external **Hardware-in-the-Loop (HIL)** Python test harness over UART.

---

## Key Highlights

- **Asymmetric Multiprocessing (AMP):** Strict separation of safety-critical pedal force calculation (Core 1 @ 50 Hz) from non-critical HMI OLED graphics rendering (Core 0 @ 10 Hz).
- **Zero UI-Induced Jitter:** Isolates blocking I2C bus transactions (400 kHz) from the high-rate brake calculation task.
- **Priority-Inherited Mutex:** Telemetry IPC utilizes FreeRTOS mutexes with priority inheritance to prevent unbounded priority inversion.
- **Fail-Safe Containment (ISO 26262 Concept):** Continuous plausibility checks detect sensor open-wire and ADC rail saturation ($|\text{RAW}| > 8{,}000{,}000$), clamping brake demand to $0\%$ in $<20\text{ ms}$.
- **Digital Signal Pipeline:** Boot-time dynamic tare calibration, deadband noise suppression ($15{,}000\text{ counts}$) against phantom braking, and Exponential Moving Average ($\alpha = 0.25$) filtering.
- **Automated HIL Test Suite:** Real-time Python test harness verifying baseline null stability, transfer function monotonicity, and dynamic deflection thresholds.

---

## System Architecture

```text
                  ┌────────────────────────────────────────────────────────┐
                  │                 ESP32 Dual-Core ECU                    │
                  │                                                        │
[Strain Gauge] ──>│  CORE 1 (Priority 5, 50 Hz / 20 ms Cycle):             │
 [HX711 24-bit]   │  • Low-level Bit-Bang Driver (GPIO 16 DT, GPIO 17 SCK) │
                  │  • 24-bit Two's Complement Sign Extension              │
                  │  • Boot-time Tare Offset Gating (20 samples)           │
                  │  • EMA Low-Pass Filter (alpha = 0.25)                  │
                  │  • Rail Saturation Check (|RAW| > 8,000,000)          │
                  │  • Safe-State Clamping (Fault -> 0% demand)            │
                  │  • Mutex Write Lock                                    │
                  │           │                                            │
                  │           ▼                                            │
                  │    [g_telemetry_mutex] ◄── Priority Inheritance        │
                  │           ▲                                            │
                  │           │                                            │
                  │  CORE 0 (Priority 1, 10 Hz / 100 ms Cycle):            │
                  │  • Mutex Read Snapshot (Non-blocking)                  │
                  │  • Dynamic 128x64 Framebuffer Render                   │
                  │  • I2C Display Flush (400 kHz) ──> [SSD1306 OLED]      │
                  └─────────────────────────┬──────────────────────────────┘
                                            │ UART @ 115200 Baud
                                            ▼
                           ┌─────────────────────────────────┐
                           │   Automated Python HIL Runner   │
                           │   • TC-01: Baseline Null Stability │
                           │   • TC-02: Transfer Monotonicity   │
                           │   • TC-03: Sensor Fault Safe-State │
                           └─────────────────────────────────┘
```

### FreeRTOS Core Mapping & Concurrency

| Task Identifier | Pinned Core | Priority | Rate / Period | Primary Role | Fault Handling |
| :--- | :---: | :---: | :---: | :--- | :--- |
| `bbw_control_task` | **Core 1** | **5** (High) | **50 Hz** (20 ms) | Bit-bang ADC acquisition, EMA filtering, deadband gating, transfer mapping | Clamps output to `0%` demand upon plausibility trip |
| `display_task` | **Core 0** | **1** (Low) | **10 Hz** (100 ms) | Mutex telemetry snapshot, OLED frame composition, 400 kHz I2C flush | Renders hatched hazard pattern during active faults |

> **Concurrency Design Rationale:**  
> A FreeRTOS Mutex (`xSemaphoreCreateMutex`) guards the shared `bbw_telemetry_t` container. Unlike simple binary semaphores, FreeRTOS mutexes implement **Priority Inheritance**. If an intermediate priority task preempts the display task while holding the telemetry lock, the display task's priority is temporarily elevated to prevent **Unbounded Priority Inversion**, guaranteeing Core 1 meets its strict 20 ms deadline.

---

## Signal Conditioning & Safety Pipeline

```text
[Raw 24-Bit ADC Counts]
          │
          ▼
┌────────────────────────────────────────┐
│  24-Bit Sign Extension (Bit 23 Pad)    │ --> Fixes two's complement sign
└───────────────────┬────────────────────┘
                    │
                    ▼
┌────────────────────────────────────────┐
│ Plausibility Check: |RAW| > 8,000,000  ├─► [FAULTSAFE: 0% Demand + Hazard Alert]
└───────────────────┬────────────────────┘
                    │ (Valid Signal)
                    ▼
┌────────────────────────────────────────┐
│ Baseline Tare Offset Subtraction       │ --> 20-sample boot average
└───────────────────┬────────────────────┘
                    │
                    ▼
┌────────────────────────────────────────┐
│ Exponential Moving Average Filter      │ --> y[n] = 0.25*x[n] + 0.75*y[n-1]
└───────────────────┬────────────────────┘
                    │
                    ▼
┌────────────────────────────────────────┐
│ Deadband Suppression (15,000 Counts)   │ --> Eliminates foot resting / vibration
└───────────────────┬────────────────────┘
                    │
                    ▼
┌────────────────────────────────────────┐
│ Linear Transfer Function Normalization │ --> [0, MAX_STROKE - DEADBAND] -> 0-100%
└───────────────────┬────────────────────┘
                    │
                    ▼
        [Commanded Brake Effort: 0-100%]
```

1. **24-Bit Two's Complement Sign Extension:**  
   The HX711 yields 24-bit signed data. Bit-bang logic checks bit 23 (`data & 0x800000`) and manually pads the upper byte (`data |= 0xFF000000`), preventing negative pedal transients from overflowing to $+16.7\text{M}$ counts.
2. **Dynamic Calibration (Tare):**  
   At boot, the ECU samples 20 consecutive readings at 20 ms intervals to establish the mechanical preload zero-point.
3. **Plausibility & Open-Circuit Failsafe:**  
   If the ADC saturates to hardware rails ($|RAW| > 8{,}000{,}000\text{ counts}$), indicating a disconnected load cell, severed harness, or bridge power failure, the ECU trips `fault_active = true` and commands **Safe State (0% brake demand)** within one cycle ($\le 20\text{ ms}$).
4. **Exponential Moving Average (EMA) Filter:**  
   A 1st-order low-pass digital filter ($\alpha = 0.25$) attenuates electromagnetic noise and pedal mechanical shudder while preserving crisp transient response:
   $$y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1]$$
5. **Deadband Threshold ($15{,}000\text{ counts}$):**  
   Guards against driver foot resting or vehicle chassis vibration, eliminating unintentional **Phantom Braking**.
6. **Transfer Function:**  
   Linear normalization scales force above deadband up to full stroke ($210{,}000\text{ counts}$):
   $$\text{Brake Effort (\%)} = \text{clamp}\left(\frac{\text{Net} - \text{Deadband}}{\text{MaxStroke} - \text{Deadband}} \times 100,\ 0,\ 100\right)$$

---

## Hardware Interfacing & Pin Map

| Subsystem | Signal Name | ESP32 GPIO | Description / Operating Parameter |
| :--- | :--- | :---: | :--- |
| **HX711 Load Cell ADC** | `HX711_DT_PIN` | **GPIO 16** | Serial Data Input (Bit-banged input) |
| | `HX711_SCK_PIN` | **GPIO 17** | Serial Clock Output (Bit-banged pulse train) |
| **SSD1306 OLED (I2C)** | `I2C_MASTER_SDA_IO` | **GPIO 4** | I2C Data Line (Internal Pull-Up enabled) |
| | `I2C_MASTER_SCL_IO` | **GPIO 15** | I2C Clock Line (400 kHz Fast-Mode) |
| **Telemetry & HIL** | `UART0_TXD` | **GPIO 1** | Telemetry Stream (`115200 Baud, 8N1`) |
| | `UART0_RXD` | **GPIO 3** | Diagnostic / Command Interface |

---

## Automated Hardware-in-the-Loop (HIL) Validation

Validation is handled by an automated external test harness ([hil_test_runner.py](file:///c:/Users/vinay/OneDrive/Documents/PlatformIO/Projects/ecu_firmware/test/tests/hil_test_runner.py)) communicating over UART at 115200 baud. The suite parses real-time telemetry frames and verifies deterministic ECU behavior against strict automotive criteria.

### Telemetry Stream Format
```text
I (1040) BBW_ECU: RAW: 15420 | NET: 420 | BRAKE: 0%
I (1060) BBW_ECU: RAW: 89310 | NET: 74310 | BRAKE: 30%
I (1080) BBW_ECU: RAW: 8388607 | NET: 8373187 | BRAKE: 0% [FAULTSAFE]
```

### Automated Test Cases

1. **TC-01: Null Baseline Verification & Noise Plausibility**
   - Collects 50 consecutive telemetry frames under zero load.
   - **Assertion:** Asserts that `pct == 0` and `fault == false` across all samples. Guarantees deadband suppression prevents phantom braking.
2. **TC-02: Dynamic Stroke & Transfer Function Validation**
   - Monitors live continuous force ramp over a 6-second window.
   - **Assertion:** Verifies monotonic pedal tracking and ensures peak commanded brake effort reaches $\ge 70\%$ threshold under manual compression.

### Sample Test Bench Execution Log

```text
============================================================
 AUTOMOTIVE HIL VALIDATION SUITE: BRAKE-BY-WIRE ECU
============================================================

[RUNNING] TC-01: Zero-Load Baseline & Noise Plausibility (50 frames)...
  -> TC-01 PASSED: Baseline is stable at 0% demand.

[ACTION REQUIRED] TC-02: Transfer Function Test.
>>> Squeeze the load cell firmly within the next 5 seconds! <<<
  Live Stroke: 84% | Raw: 198420
  -> Peak Commanded Brake Effort: 84%
  -> TC-02 PASSED: Dynamic deflection registered successfully.

============================================================
 HIL RESULT: TC-01: PASS | TC-02: PASS
============================================================
```

---

## Repository Structure

```text
ecu_firmware/
├── include/
│   ├── README
│   └── ecu_types.h             # Telemetry & state machine data structures
├── src/
│   ├── CMakeLists.txt          # Component build configuration
│   └── main.c                  # Dual-core FreeRTOS firmware implementation
├── test/
│   ├── README
│   └── tests/
│       ├── hil_test_runner.py  # Automated Python HIL test harness
│       └── hil_report.txt      # Automated test bench execution logs
├── CMakeLists.txt              # Top-level ESP-IDF CMake definition
├── platformio.ini              # PlatformIO environment (esp32dev, espidf, 115200)
├── sdkconfig.esp32dev          # ESP-IDF target configuration
└── README.md                   # System documentation
```

---

## Getting Started

### Prerequisites

- [PlatformIO Core (CLI)](https://platformio.org/) or VSCode PlatformIO IDE extension
- ESP-IDF Framework (handled automatically by PlatformIO)
- Python 3.9+ with `pyserial`:
  ```bash
  pip install pyserial
  ```

### 1. Build and Flash Firmware

Connect your ESP32 board to your development host (default COM port: `COM3`):

```bash
# Build firmware
pio run

# Flash to ESP32
pio run --target upload

# Open serial monitor
pio device monitor --baud 115200
```

### 2. Execute Automated HIL Validation Suite

With the ESP32 active on `COM3`:

```bash
python test/tests/hil_test_runner.py
```

Follow on-screen instructions during **TC-02** to verify dynamic force transducer actuation.

---

## Future Roadmap & Safety Enhancements

- [ ] **CAN-FD / TWAI Integration (ISO 11898-1):** Transmit cyclic 10 ms brake demand frames (`0x110`) with rolling counter and CRC-8 checksum to wheel actuators.
- [ ] **Dual-Sensor Redundancy (ASIL-D Concept):** Implement redundant Dual-Hall / Strain-gauge channel cross-checking with 10% discrepancy trip threshold.
- [ ] **Dynamic Pedal Mapping:** Configurable pedal response curves (Linear Standard vs. Progressive Sport brake curve).

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.