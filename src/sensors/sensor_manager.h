#pragma once
#include "sensor_interface.h"
#include "sensors/imu_sensor.h"
#include "fsr_sensor.h"
#include <functional>

// Generic sensor data structure
struct SensorData {
    bool valid;
    unsigned long timestamp;
    
    SensorData() : valid(false), timestamp(0) {}
};

class SimpleSensorManager {
private:
    static const int MAX_SENSORS = 10;  // Enough for multiple sensors
    Sensor* sensors_[MAX_SENSORS];
    int sensorCount_;
    unsigned long lastUpdate_;

public:
    SimpleSensorManager();

    // Sensor management
    bool addSensor(Sensor* sensor);
    bool initializeAll();
    void updateAll();

    // Data access
    bool areAllSensorsReady() const;
    Sensor* getSensor(const char* name) const;
    int getSensorCount() const;
    Sensor* getSensorByIndex(int index) const;

    // Type-specific access
    int getSensorsOfType(const char* typePrefix, Sensor** resultArray, int maxResults) const;
    int getAllFSRSensors(FSRSensor** fsrArray, int maxResults) const;
    int getAllIMUSensors(MPU6050Sensor** imuArray, int maxResults) const;

    // Utility functions
    void printStatus() const;
    void printAllSensorData() const;
    
    // Event callback system
    using SensorEventCallback = std::function<void(const char*, const SensorData&)>;
    void setEventCallback(SensorEventCallback callback);
    
    // Health checking
    bool areAllSensorsHealthy() const;
};