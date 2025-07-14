#pragma once
#include <Arduino.h>
#include <math.h>
#include "sensor_interface.h"

// Simple structure for IMU data
struct IMUData {
    float accelX, accelY, accelZ;  // g-force
    float gyroX, gyroY, gyroZ;     // degrees/second
    float temperature;             // Celsius
    unsigned long timestamp;
    bool valid;
    
    IMUData() : accelX(0), accelY(0), accelZ(0), 
                gyroX(0), gyroY(0), gyroZ(0), 
                temperature(0), timestamp(0), valid(false) {}
};

// MPU6050 IMU Sensor
class MPU6050 : public Sensor {
private:
    static const uint8_t MPU6050_ADDRESS = 0x68;
    static const uint8_t PWR_MGMT_1 = 0x6B;
    static const uint8_t ACCEL_XOUT_H = 0x3B;
    
    const char* name_;
    IMUData data_;
    bool initialized_;

public:
    MPU6050(const char* name = "MPU6050");

    bool begin() override;
    bool update() override;
    bool isReady() const override;
    const char* getName() const override;

    IMUData getData() const;
    
    // Helper functions for common calculations
    float getAccelMagnitude() const;
    float getRoll() const;
    float getPitch() const;
};