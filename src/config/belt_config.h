#pragma once
#include <Arduino.h>

namespace BeltConfig {
    // Hardware pins
    constexpr int PRESSURE_SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
    constexpr int IMU_SDA_PIN = 11; // A4
    constexpr int IMU_SCL_PIN = 12; // A5
    constexpr int LED_PIN = 2;
    constexpr int HAPTIC_PIN = 4;
    
    // Timing constants
    constexpr unsigned long SENSOR_UPDATE_INTERVAL_MS = 50;
    constexpr unsigned long PAIRING_TIMEOUT_MS = 300000;
    constexpr unsigned long INACTIVITY_TIMEOUT_MS = 600000;
    constexpr unsigned long FEEDBACK_DURATION_MS = 2000;
    
    // Calibration constants
    constexpr int BASELINE_SAMPLES = 100;
    constexpr int BRACE_SAMPLES = 50;
    constexpr float BRACE_THRESHOLD = 0.8f;
    
    // // Bluetooth
    // constexpr char DEVICE_NAME[] = "SmartBelt";
    // constexpr char SERVICE_UUID[] = "12345678-1234-1234-1234-123456789abc";
}