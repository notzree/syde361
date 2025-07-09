#include "force_sensor.h"

ForceSensor::ForceSensor(const char* name, int pin) 
    : name_(name), pin_(pin), healthy_(false) {}

bool ForceSensor::initialize() {
    pinMode(pin_, INPUT);
    // A simple check to see if the pin is valid, you might do more here
    if (pin_ >= 0) {
        healthy_ = true;
        return true;
    }
    return false;
}

bool ForceSensor::update() {
    if (!healthy_) return false;

    // Read the analog value from the FSR
    int rawValue = analogRead(pin_);
    
    // Populate the data structure
    data_.values[0] = static_cast<float>(rawValue);
    data_.count = 1;
    data_.timestamp = millis();
    data_.valid = true;
    
    return true;
}

SensorData ForceSensor::getData() const {
    return data_;
}

bool ForceSensor::isHealthy() const {
    return healthy_;
}

const char* ForceSensor::getName() const {
    return name_;
}