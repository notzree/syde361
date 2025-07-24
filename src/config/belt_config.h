#pragma once
#include <Arduino.h>

namespace BeltConfig
{
    // Hardware pins
    // constexpr int PRESSURE_SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
    constexpr int IMU_SDA_PIN = 11; // A4
    constexpr int IMU_SCL_PIN = 12; // A5
    constexpr int LED_PIN = 2;
    constexpr int HAPTIC_PIN = 4;

    // Timing constants
    constexpr unsigned long SENSOR_UPDATE_INTERVAL_MS = 50;
    constexpr unsigned long PAIRING_TIMEOUT_MS = 300000;
    constexpr unsigned long INACTIVITY_TIMEOUT_MS = 600000;
    constexpr unsigned long FEEDBACK_DURATION_MS = 2000;

    // Thresholds for detection algorithms
    const float GOOD_DISTRIBUTION_THRESHOLD = 0.15; // Low variance for even distribution
    const float ASYMMETRY_THRESHOLD = 0.3;          // Max acceptable left/right asymmetry
    const float PITCH_THRESHOLD = 8.0;              // Degrees from neutral
    const float ROLL_THRESHOLD = 15.0;              // Degrees from neutral
    const float DANGER_PITCH_THRESHOLD = 10.0;      // Dangerous side lean
    const float DANGER_ROLL_THRESHOLD = 50.0;       // Dangerous forward/back lean
    const float GOOD_BRACE_RATIO = 1.2;             // Pressure must be 150% of baseline
    const float BEND_PITCH_THRESHOLD = 15.0;
    const float MIN_ENGAGEMENT_WEIGHT = 10.0; // Minimum weight to consider engaged

    // Timing parameters
    const unsigned long BUTTON_DOUBLE_PRESS_WINDOW = 500; // ms
    const unsigned long CALIBRATION_DURATION = 5000;      // ms
    const unsigned long FEEDBACK_GOOD_DURATION = 300;     // ms
    const unsigned long FEEDBACK_BAD_DURATION = 800;      // ms
    const unsigned long EVENT_CHECK_INTERVAL = 50;        // ms (20Hz)

    // Button debouncing
    const unsigned long DEBOUNCE_DELAY = 50; // ms

    // Calibration constants
    constexpr int BASELINE_SAMPLES = 100;
    constexpr int BRACE_SAMPLES = 50;
    constexpr float BRACE_THRESHOLD = 0.8f;

    static const float MIN_ENGAGEMENT_PRESSURE = 600;

    // Need to update these, not sure if they make sense
    constexpr int CALIBRATION_SAMPLE_INTERVAL = 10;
    constexpr int CALIBRATION_SAMPLE_COUNT = 10;
}