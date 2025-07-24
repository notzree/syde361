#include "fsr_sensor.h"
#include <Arduino.h>

// Configuration constants for 12-bit ADC (ESP32)
static const int ADC_RESOLUTION = 4095; // 12-bit ADC max value
static const float VCC = 3.3;           // ESP32 supply voltage

FSRSensor::FSRSensor(const char *name, int pin)
    : name_(name), pin_(pin), initialized_(false)
{
    // Initialize baseline and smoothing
    baseline_ = 0;
    for (int i = 0; i < SMOOTHING_SAMPLES; i++)
    {
        smoothingBuffer_[i] = 0;
    }
    smoothingIndex_ = 0;
    smoothingTotal_ = 0;
}

bool FSRSensor::begin()
{
    pinMode(pin_, INPUT);
    initialized_ = true;

    // Auto-calibrate baseline by taking several readings when no load is applied
    // In pull-down config, no load = high ADC values (near ADC_RESOLUTION)
    Serial.print("Calibrating ");
    Serial.print(name_);
    Serial.println(" baseline (ensure no load applied)...");

    delay(1000); // Give user time to remove any load

    long total = 0;
    const int calibrationSamples = 50;

    for (int i = 0; i < calibrationSamples; i++)
    {
        total += analogRead(pin_);
        delay(20);
    }

    baseline_ = total / calibrationSamples;

    Serial.print(name_);
    Serial.print(" baseline set to: ");
    Serial.println(baseline_);

    return true;
}

bool FSRSensor::update()
{
    if (!initialized_)
        return false;

    // Read raw ADC value
    int rawReading = analogRead(pin_);

    // Apply smoothing using rolling average
    smoothingTotal_ = smoothingTotal_ - smoothingBuffer_[smoothingIndex_];
    smoothingBuffer_[smoothingIndex_] = rawReading;
    smoothingTotal_ = smoothingTotal_ + smoothingBuffer_[smoothingIndex_];
    smoothingIndex_ = (smoothingIndex_ + 1) % SMOOTHING_SAMPLES;

    // Store smoothed raw value
    data_.rawValue = smoothingTotal_ / SMOOTHING_SAMPLES;

    // Calculate pressure relative to baseline
    // In pull-down config: baseline (no load) > current reading (with load)
    // So pressure = baseline - current reading
    float pressureAboveBaseline = baseline_ - data_.rawValue;
    if (pressureAboveBaseline < 0)
    {
        pressureAboveBaseline = 0; // Clamp to zero (reading higher than baseline)
    }

    // Store as weight (pressure units above baseline)
    data_.weight = pressureAboveBaseline;

    data_.timestamp = millis();
    data_.valid = true;
    return true;
}

void FSRSensor::setBaseline(int baseline)
{
    baseline_ = baseline;
    Serial.print(name_);
    Serial.print(" baseline manually set to: ");
    Serial.println(baseline_);
}

int FSRSensor::getBaseline() const
{
    return baseline_;
}

int FSRSensor::getRawValue() const
{
    return data_.rawValue;
}

float FSRSensor::getPressureAboveBaseline() const
{
    return data_.weight; // weight field now stores pressure above baseline
}

bool FSRSensor::isReady() const
{
    return initialized_ && data_.valid;
}

const char *FSRSensor::getName() const
{
    return name_;
}

FSRData FSRSensor::getData() const
{
    return data_;
}

bool FSRSensor::isPressed(float threshold) const
{
    // Check if pressure above baseline exceeds threshold
    // In pull-down config, this means: (baseline - current_reading) > threshold
    return data_.valid && data_.weight > threshold;
}

bool FSRSensor::isBracing(float threshold) const
{
    // More descriptive method name for bracing detection
    return isPressed(threshold);
}

// Additional helper methods you might find useful:

float FSRSensor::getPressurePercentage() const
{
    // Returns pressure as percentage of full scale
    // Full scale assumed to be baseline (no load) to near 0 (max load)
    if (baseline_ <= 0)
        return 0;

    float percentage = (data_.weight * 100.0) / baseline_;
    return min(percentage, static_cast<float>(100.0)); // Cap at 100%
}

float FSRSensor::getVoltage() const
{
    // Convert raw ADC to voltage
    return (data_.rawValue * VCC) / ADC_RESOLUTION;
}

float FSRSensor::getResistance() const
{
    // Calculate FSR resistance using voltage divider equation
    // For pull-down config: R_fsr = R_pullup * V_out / (VCC - V_out)
    const float R_PULLUP = 330.0; // 10K ohm
    float voltage = getVoltage();

    if (voltage < (VCC - 0.01))
    { // Avoid division by zero
        return R_PULLUP * voltage / (VCC - voltage);
    }
    else
    {
        return 999999; // Very high resistance (no pressure)
    }
}

void FSRSensor::printDebugInfo() const
{
    Serial.print(name_);
    Serial.print(" - Raw: ");
    Serial.print(data_.rawValue);
    Serial.print(", Baseline: ");
    Serial.print(baseline_);
    Serial.print(", Pressure: ");
    Serial.print(data_.weight);
    Serial.print(", Voltage: ");
    Serial.print(getVoltage(), 3);
    Serial.print("V, Resistance: ");

    float resistance = getResistance();
    if (resistance > 100000)
    {
        Serial.print(">100K ohm");
    }
    else
    {
        Serial.print(resistance, 0);
        Serial.print(" ohm");
    }

    Serial.print(", Pressed: ");
    Serial.println(isPressed(50) ? "YES" : "NO"); // 50 ADC units threshold
}