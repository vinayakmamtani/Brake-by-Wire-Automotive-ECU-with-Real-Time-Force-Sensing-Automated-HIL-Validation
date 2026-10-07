#pragma once
#include <Arduino.h>

enum EcuState : uint8_t {
    STATE_IDLE       = 0,
    STATE_CRUISING   = 1,
    STATE_BRAKING    = 2,
    STATE_TCS_ACTIVE = 3,
    STATE_ABS_ACTIVE = 4,
    STATE_SAFE_STOP  = 5
};

struct VehicleTelemetry {
    EcuState state;
    float rpm_left;
    float rpm_right;
    float slip_ratio;
    long brake_load_g;
    uint32_t loop_latency_us;
    uint32_t timestamp_ms;
};