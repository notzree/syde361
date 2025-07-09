#include "sensor_manager.h"
#include <Arduino.h>

void SensorManager::addSensor(std::unique_ptr<ISensor> sensor) {
    sensors_.push_back(std::move(sensor));
}

bool SensorManager::initializeAll() {
    Serial.println("Initializing all sensors...");
    for (auto& sensor : sensors_) {
        if (!sensor->initialize()) {
            Serial.print("Failed to initialize sensor: ");
            Serial.println(sensor->getName());
            return false;
        }
    }
    Serial.println("All sensors initialized successfully");
    return true;
}

void SensorManager::updateAll() {
    unsigned long currentTime = millis();
    
    for (auto& sensor : sensors_) {
        if (sensor->update()) {
            SensorData data = sensor->getData();
            if (data.valid && eventCallback_) {
                eventCallback_(sensor->getName(), data);
            }
        }
    }
    
    lastUpdate_ = currentTime;
}

SensorData SensorManager::getSensorData(const char* sensorName) const {
    for (const auto& sensor : sensors_) {
        if (strcmp(sensor->getName(), sensorName) == 0) {
            return sensor->getData();
        }
    }
    return SensorData(); // Return invalid data if sensor not found
}

std::vector<SensorData> SensorManager::getAllSensorData() const {
    std::vector<SensorData> allData;
    allData.reserve(sensors_.size());
    
    for (const auto& sensor : sensors_) {
        allData.push_back(sensor->getData());
    }
    
    return allData;
}

bool SensorManager::areAllSensorsHealthy() const {
    for (const auto& sensor : sensors_) {
        if (!sensor->isHealthy()) {
            return false;
        }
    }
    return true;
}

void SensorManager::setEventCallback(SensorEventCallback callback) {
    eventCallback_ = callback;
} 