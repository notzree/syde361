#include "sensors/imu_sensor.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

MPU6050::MPU6050(const char* name) : name_(name), initialized_(false) {}

bool MPU6050::begin() {
    Wire.begin();
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    delay(100);
    initialized_ = true;
    return true;
}

bool MPU6050::update() {
    if (!initialized_) return false;
    
    // Read all 14 bytes of sensor data at once
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission() != 0) {
        data_.valid = false;
        return false;
    }
    
    Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14);
    if (Wire.available() != 14) {
        data_.valid = false;
        return false;
    }
    
    // Read accelerometer
    int16_t rawAccelX = (Wire.read() << 8) | Wire.read();
    int16_t rawAccelY = (Wire.read() << 8) | Wire.read();
    int16_t rawAccelZ = (Wire.read() << 8) | Wire.read();
    
    // Read temperature
    int16_t rawTemp = (Wire.read() << 8) | Wire.read();
    
    // Read gyroscope
    int16_t rawGyroX = (Wire.read() << 8) | Wire.read();
    int16_t rawGyroY = (Wire.read() << 8) | Wire.read();
    int16_t rawGyroZ = (Wire.read() << 8) | Wire.read();
    
    // Convert to meaningful units
    // Accelerometer: ±2g range, 16-bit
    data_.accelX = rawAccelX / 16384.0f;
    data_.accelY = rawAccelY / 16384.0f;
    data_.accelZ = rawAccelZ / 16384.0f;
    
    // Gyroscope: ±250°/s range, 16-bit
    data_.gyroX = rawGyroX / 131.0f;
    data_.gyroY = rawGyroY / 131.0f;
    data_.gyroZ = rawGyroZ / 131.0f;
    
    // Temperature
    data_.temperature = (rawTemp / 340.0f) + 36.53f;
    
    data_.timestamp = millis();
    data_.valid = true;
    return true;
}

bool MPU6050::isReady() const {
    return initialized_ && data_.valid;
}

const char* MPU6050::getName() const {
    return name_;
}

IMUData MPU6050::getData() const {
    return data_;
}

float MPU6050::getAccelMagnitude() const {
    if (!data_.valid) return 0;
    return sqrt(data_.accelX * data_.accelX + 
               data_.accelY * data_.accelY + 
               data_.accelZ * data_.accelZ);
}

float MPU6050::getRoll() const {
    if (!data_.valid) return 0;
    return atan2(data_.accelY, sqrt(data_.accelX * data_.accelX + data_.accelZ * data_.accelZ)) * 180.0f / PI;
}

float MPU6050::getPitch() const {
    if (!data_.valid) return 0;
    return atan2(-data_.accelX, sqrt(data_.accelY * data_.accelY + data_.accelZ * data_.accelZ)) * 180.0f / PI;
}