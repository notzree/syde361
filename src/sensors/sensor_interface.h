#pragma once

// Base sensor interface - keep it simple
class Sensor {
public:
    virtual ~Sensor() = default;
    virtual bool begin() = 0;
    virtual bool update() = 0;
    virtual bool isReady() const = 0;
    virtual const char* getName() const = 0;
};