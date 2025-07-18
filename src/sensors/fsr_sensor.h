#pragma once
#include "sensor_interface.h"

// Simple structure for FSR data
struct FSRData {
    int rawValue;          // Raw ADC reading
    unsigned long timestamp;
    bool valid;
    float weight;          // Pressure above baseline (in ADC units)
    
    FSRData() : weight(0), rawValue(0), timestamp(0), valid(false) {}
};

// Force Sensitive Resistor Sensor (Pull-down Configuration)
class FSRSensor : public Sensor {
private:
    const char* name_;
    int pin_;
    FSRData data_;
    bool initialized_;
    static const int SMOOTHING_SAMPLES = 10;
    int smoothingBuffer_[SMOOTHING_SAMPLES];
    int smoothingIndex_;
    long smoothingTotal_;
    int baseline_;  // Baseline ADC value (no load applied)

public:
    FSRSensor(const char* name, int pin);

    // Core sensor interface methods
    bool begin() override;
    bool update() override;
    bool isReady() const override;
    const char* getName() const override;

    // Data access methods
    FSRData getData() const;
    int getRawValue() const;
    float getPressureAboveBaseline() const;
    
    // Baseline management
    void setBaseline(int baseline);
    int getBaseline() const;
    
    // Pressure detection methods
    bool isPressed(float threshold = 50.0f) const;  // Default threshold in ADC units
    bool isBracing(float threshold = 100.0f) const; // Higher threshold for bracing
    
    // Additional helper methods
    float getPressurePercentage() const;  // Returns pressure as 0-100%
    float getVoltage() const;             // Get current voltage reading
    float getResistance() const;          // Calculate FSR resistance
    void printDebugInfo() const;          // Print detailed sensor info
};