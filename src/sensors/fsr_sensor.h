#pragma once
#include "sensor_interface.h"

// Simple structure for FSR data
struct FSRData {
    // float force;           // 
    int rawValue;          // Raw ADC reading
    unsigned long timestamp;
    bool valid;
    float weight;
    
    FSRData() : weight(0), rawValue(0), timestamp(0), valid(false) {}
};

// Force Sensitive Resistor Sensor
class FSRSensor : public Sensor {
private:
    const char* name_;
    int pin_;
    FSRData data_;
    bool initialized_;

public:
    FSRSensor(const char* name, int pin);

    bool begin() override;
    bool update() override;
    bool isReady() const override;
    const char* getName() const override;

    FSRData getData() const;
    
    // Helper function to check if force is being applied
    bool isPressed(float threshold = 0.1f) const;
};