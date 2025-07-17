#include "fsr_sensor.h"
#include <Arduino.h>

FSRSensor::FSRSensor(const char* name, int pin) 
    : name_(name), pin_(pin), initialized_(false) {
    // Initialize baseline and smoothing (need to add these to header)
    baseline_ = 0;
    for (int i = 0; i < SMOOTHING_SAMPLES; i++) {
        smoothingBuffer_[i] = 0;
    }
    smoothingIndex_ = 0;
    smoothingTotal_ = 0;
}

bool FSRSensor::begin() {
    pinMode(pin_, INPUT);
    initialized_ = true;
    return true;
}

bool FSRSensor::update() {
    if (!initialized_) return false;
    
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
    float pressureAboveBaseline = data_.rawValue - baseline_;
    if (pressureAboveBaseline < 0) {
        pressureAboveBaseline = 0; // Clamp to zero
    }
    
    // Store as weight (pressure units above baseline)
    data_.weight = pressureAboveBaseline;
    
    data_.timestamp = millis();
    data_.valid = true;
    return true;
}

void FSRSensor::setBaseline(int baseline) {
    baseline_ = baseline;
    Serial.print(name_);
    Serial.print(" baseline set to: ");
    Serial.println(baseline_);
}

int FSRSensor::getBaseline() const {
    return baseline_;
}

int FSRSensor::getRawValue() const {
    return data_.rawValue;
}

float FSRSensor::getPressureAboveBaseline() const {
    return data_.weight; // weight field now stores pressure above baseline
}

bool FSRSensor::isReady() const {
    return initialized_ && data_.valid;
}

const char* FSRSensor::getName() const {
    return name_;
}

FSRData FSRSensor::getData() const {
    return data_;
}

bool FSRSensor::isPressed(float threshold) const {
    // Check if pressure above baseline exceeds threshold
    return data_.valid && data_.weight > threshold;
}

bool FSRSensor::isBracing(float threshold) const {
    // More descriptive method name for bracing detection
    return isPressed(threshold);
}