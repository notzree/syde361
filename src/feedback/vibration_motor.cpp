#include "vibration_motor.h"
#include <Arduino.h>
VibrationMotor::VibrationMotor(const char* name, int pin) 
    : name_(name), pin_(pin) {}

bool VibrationMotor::initialize() {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW); // Ensure it's off initially
    return true;
}

void VibrationMotor::buzz() const {
    // A simple 150ms buzz pattern
    digitalWrite(pin_, HIGH);
    delay(150);
    digitalWrite(pin_, LOW);
}

const char* VibrationMotor::getName() const {
    return name_;
}