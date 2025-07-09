#pragma once
#include "sensor_interface.h"
#include <memory>
#include <vector>
#include <functional>
#include <cstring>

class SensorManager {
public:
    using SensorEventCallback = std::function<void(const char*, const SensorData&)>;
    
    SensorManager() = default;
    ~SensorManager() = default;
    
    // Sensor management
    void addSensor(std::unique_ptr<ISensor> sensor);
    bool initializeAll();
    void updateAll();
    
    // Data access
    SensorData getSensorData(const char* sensorName) const;
    std::vector<SensorData> getAllSensorData() const;
    bool areAllSensorsHealthy() const;
    
    // Event handling
    void setEventCallback(SensorEventCallback callback);
    
private:
    std::vector<std::unique_ptr<ISensor>> sensors_;
    SensorEventCallback eventCallback_;
    unsigned long lastUpdate_;
};