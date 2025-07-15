// sensors/imu_sensor.h
#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "sensors/sensor_interface.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>

struct IMUData {
    float accelX, accelY, accelZ;    // in g-force
    float gyroX, gyroY, gyroZ;       // in degrees/s
    float temperature;               // in Celsius
    unsigned long timestamp;         // in milliseconds
    bool valid;                      // data validity flag
};

class MPU6050Sensor : public Sensor{
public:
    MPU6050Sensor(const char* name);
    
    bool begin();
    bool update();
    bool isReady() const;
    const char* getName() const;
    
    IMUData getData() const;
    float getAccelMagnitude() const;
    float getRoll() const;
    float getPitch() const;

private:
    Adafruit_MPU6050 mpu_;
    const char* name_;
    bool initialized_;
    IMUData data_;
};

#endif // IMU_SENSOR_H