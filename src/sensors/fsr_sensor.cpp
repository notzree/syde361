#include "fsr_sensor.h"
#include <Arduino.h>

FSRSensor::FSRSensor(const char* name, int pin) : name_(name), pin_(pin), initialized_(false) {}

bool FSRSensor::begin() {
    pinMode(pin_, INPUT);
    initialized_ = true;
    return true;
}

bool FSRSensor::update() {
    if (!initialized_) return false;
    
    data_.rawValue = analogRead(pin_);
    
    // Convert to normalized force (0.0 to 1.0)
    // Adjust this mapping based on your FSR characteristics
    data_.force = data_.rawValue / 1023.0f;
    
    data_.timestamp = millis();
    data_.valid = true;
    return true;
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
    return data_.valid && data_.force > threshold;
}