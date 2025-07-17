#include "sensors/imu_sensor.h"
#include "../config/belt_config.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <math.h>

// Renamed class to avoid conflicts with Adafruit library
MPU6050Sensor::MPU6050Sensor(const char* name) 
    : name_(name), initialized_(false) {}

bool MPU6050Sensor::begin() {
    Serial.print(F("MPU6050Sensor: Starting initialization for "));
    Serial.println(name_);
    
    if (!mpu_.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        Serial.println("MPU6050Sensor: Initialization failed - returning false");
        return false;
    }
    Serial.println(F("MPU6050Sensor: Found MPU6050!"));
    // Configure the sensor (matching the working demo)
    mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu_.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    
    mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu_.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu_.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }
    
    // Add a small delay after configuration
    delay(100);
    
    initialized_ = true;
    Serial.println(F("MPU6050Sensor: Initialization completed successfully!"));
    return true;
}

bool MPU6050Sensor::update() {
    if (!initialized_) {
        Serial.println(F("MPU6050Sensor: Not initialized"));
        return false;
    }
    
    // Get new sensor events with the readings
    sensors_event_t accel, gyro, temp;
    
    if (!mpu_.getEvent(&accel, &gyro, &temp)) {
        Serial.println(F("MPU6050Sensor: Failed to read sensor data"));
        data_.valid = false;
        return false;
    }
    
    // Convert to your IMUData format
    // Acceleration is already in m/s^2, convert to g-force
    data_.accelX = accel.acceleration.x / 9.81f;
    data_.accelY = accel.acceleration.y / 9.81f;
    data_.accelZ = accel.acceleration.z / 9.81f;
    
    // Gyro is in rad/s, convert to degrees/s
    data_.gyroX = gyro.gyro.x * 180.0f / PI;
    data_.gyroY = gyro.gyro.y * 180.0f / PI;
    data_.gyroZ = gyro.gyro.z * 180.0f / PI;
    
    // Temperature is already in Celsius
    data_.temperature = temp.temperature;
    
    data_.timestamp = millis();
    data_.valid = true;
    
    return true;
}

bool MPU6050Sensor::isReady() const {
    return initialized_ && data_.valid;
}

const char* MPU6050Sensor::getName() const {
    return name_;
}

IMUData MPU6050Sensor::getData() const {
    return data_;
}

float MPU6050Sensor::getAccelMagnitude() const {
    if (!data_.valid) return 0;
    return sqrt(data_.accelX * data_.accelX + 
                data_.accelY * data_.accelY + 
                data_.accelZ * data_.accelZ);
}

float MPU6050Sensor::getRoll() const {
    if (!data_.valid) return 0;
    return atan2(data_.accelY, data_.accelZ) * 180.0f / PI;
}

float MPU6050Sensor::getPitch() const {
    if (!data_.valid) return 0;
    return atan2(-data_.accelX, sqrt(data_.accelY * data_.accelY + data_.accelZ * data_.accelZ)) * 180.0f / PI;
}