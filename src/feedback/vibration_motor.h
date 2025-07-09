#pragma once
#include "motor_interface.h"
#include <Arduino.h>

class VibrationMotor : public IMotor {
public:
    VibrationMotor(const char* name, int pin);
    virtual ~VibrationMotor() = default;

    bool initialize() override;
    void buzz() const override;
    const char* getName() const override;

private:
    const char* name_;
    int pin_;
};