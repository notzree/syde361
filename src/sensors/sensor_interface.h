#pragma once
#include <functional>
#include <vector>

struct SensorData {
    float values[16];  // Max 16 sensor values
    size_t count;
    unsigned long timestamp;
    bool valid;
    
    SensorData() : count(0), timestamp(0), valid(false) {
        for (int i = 0; i < 16; i++) values[i] = 0.0f;
    }
};

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool initialize() = 0;
    virtual bool update() = 0;
    virtual SensorData getData() const = 0;
    virtual bool isHealthy() const = 0;
    virtual const char* getName() const = 0;
};

// gpio sensor

// bluetooth sensor