#include "sensors/sensor_manager.h"
#include "sensors/imu_sensor.h"
#include "sensors/sensor_interface.h"
#include <Arduino.h>
#include <string.h>

SimpleSensorManager::SimpleSensorManager() : sensorCount_(0), lastUpdate_(0) {
    for (int i = 0; i < MAX_SENSORS; i++) {
        sensors_[i] = nullptr;
    }
}

bool SimpleSensorManager::addSensor(Sensor* sensor) {
    if (sensorCount_ < MAX_SENSORS && sensor != nullptr) {
        sensors_[sensorCount_] = sensor;
        sensorCount_++;
        return true;
    }
    return false;
}

bool SimpleSensorManager::initializeAll() {
    Serial.println("Initializing all sensors...");
    for (int i = 0; i < sensorCount_; i++) {
        if (!sensors_[i]->begin()) {
            Serial.print("Failed to initialize sensor: ");
            Serial.println(sensors_[i]->getName());
            return false;
        }
        Serial.print("Initialized: ");
        Serial.println(sensors_[i]->getName());
    }
    Serial.println("All sensors initialized successfully");
    return true;
}

void SimpleSensorManager::updateAll() {
    for (int i = 0; i < sensorCount_; i++) {
        sensors_[i]->update();
    }
    lastUpdate_ = millis();
}

bool SimpleSensorManager::areAllSensorsReady() const {
    for (int i = 0; i < sensorCount_; i++) {
        if (!sensors_[i]->isReady()) {
            return false;
        }
    }
    return true;
}

Sensor* SimpleSensorManager::getSensor(const char* name) const {
    for (int i = 0; i < sensorCount_; i++) {
        if (strcmp(sensors_[i]->getName(), name) == 0) {
            return sensors_[i];
        }
    }
    return nullptr;
}

int SimpleSensorManager::getSensorCount() const {
    return sensorCount_;
}

Sensor* SimpleSensorManager::getSensorByIndex(int index) const {
    if (index >= 0 && index < sensorCount_) {
        return sensors_[index];
    }
    return nullptr;
}

int SimpleSensorManager::getSensorsOfType(const char* typePrefix, Sensor** resultArray, int maxResults) const {
    int found = 0;
    for (int i = 0; i < sensorCount_ && found < maxResults; i++) {
        if (strncmp(sensors_[i]->getName(), typePrefix, strlen(typePrefix)) == 0) {
            resultArray[found] = sensors_[i];
            found++;
        }
    }
    return found;
}

int SimpleSensorManager::getAllFSRSensors(FSRSensor** fsrArray, int maxResults) const {
    int found = 0;
    for (int i = 0; i < sensorCount_ && found < maxResults; i++) {
        // Check if this sensor's name contains "FSR"
        if (strstr(sensors_[i]->getName(), "FSR") != nullptr) {
            fsrArray[found] = static_cast<FSRSensor*>(sensors_[i]);
            found++;
        }
    }
    return found;
}

int SimpleSensorManager::getAllIMUSensors(MPU6050Sensor** imuArray, int maxResults) const {
    int found = 0;
    for (int i = 0; i < sensorCount_ && found < maxResults; i++) {
        // Check if this sensor's name contains "IMU"
        if (strstr(sensors_[i]->getName(), "IMU") != nullptr) {
            imuArray[found] = static_cast<MPU6050Sensor*>(sensors_[i]);
            found++;
        }
    }
    return found;
}

void SimpleSensorManager::printStatus() const {
    Serial.println("=== Sensor Status ===");
    for (int i = 0; i < sensorCount_; i++) {
        Serial.print(sensors_[i]->getName());
        Serial.print(": ");
        Serial.println(sensors_[i]->isReady() ? "Ready" : "Not Ready");
    }
    Serial.println("====================");
}

void SimpleSensorManager::printAllSensorData() const {
    Serial.println("=== All Sensor Data ===");
    
    // Print all FSR data
    FSRSensor* fsrSensors[5];  // Max 5 FSR sensors
    int fsrCount = getAllFSRSensors(fsrSensors, 5);
    if (fsrCount > 0) {
        Serial.println("FSR Sensors:");
        for (int i = 0; i < fsrCount; i++) {
            if (fsrSensors[i]->isReady()) {
                FSRData data = fsrSensors[i]->getData();
                Serial.print("  ");
                Serial.print(fsrSensors[i]->getName());
                Serial.print(": ");
                Serial.print(data.weight, 3);
                Serial.print(" (");
                Serial.print(data.rawValue);
                Serial.print(")");
                if (fsrSensors[i]->isPressed()) {
                    Serial.print(" [PRESSED]");
                }
                Serial.println();
            }
        }
    }
    
    // Print all IMU data
    MPU6050Sensor* imuSensors[5];  // Max 5 IMU sensors
    int imuCount = getAllIMUSensors(imuSensors, 5);
    if (imuCount > 0) {
        Serial.println("IMU Sensors:");
        for (int i = 0; i < imuCount; i++) {
            if (imuSensors[i]->isReady()) {
                IMUData data = imuSensors[i]->getData();
                Serial.print("  ");
                Serial.print(imuSensors[i]->getName());
                Serial.print(": Roll=");
                Serial.print(imuSensors[i]->getRoll(), 1);
                Serial.print("°, Pitch=");
                Serial.print(imuSensors[i]->getPitch(), 1);
                Serial.print("°, AccelMag=");
                Serial.print(imuSensors[i]->getAccelMagnitude(), 2);
                Serial.print("g, Temp=");
                Serial.print(data.temperature, 1);
                Serial.println("°C");
            }
        }
    }
    Serial.println("=======================");
}

void SimpleSensorManager::setEventCallback(SensorEventCallback callback) {
    // For now, this is a placeholder implementation
    // The actual callback functionality would need to be implemented
    // based on your specific requirements
}

bool SimpleSensorManager::areAllSensorsHealthy() const {
    // Check if all sensors are ready and functioning
    for (int i = 0; i < sensorCount_; i++) {
        if (!sensors_[i]->isReady()) {
            return false;
        }
    }
    return true;
}