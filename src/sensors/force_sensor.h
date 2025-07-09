#pragma once
#include "sensor_interface.h"
#include <Arduino.h>

class ForceSensor : public ISensor {
public:
    ForceSensor(const char* name, int pin);
    virtual ~ForceSensor() = default;

    bool initialize() override;
    bool update() override;
    SensorData getData() const override;
    bool isHealthy() const override;
    const char* getName() const override;

private:
    const char* name_;
    int pin_;
    SensorData data_;
    bool healthy_;
};