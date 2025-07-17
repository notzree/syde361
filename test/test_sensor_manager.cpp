#include <Arduino.h>
#include "sensors/sensor_manager.h"
#include "sensors/imu_sensor.h"
#include "sensors/fsr_sensor.h"


// Pin definitions (adjust as needed)
#define IMU_NAME "IMU"
#define FSR_NAME "FSR"
#define FSR_PIN A0

SimpleSensorManager sensorManager;

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial to connect
    Serial.println("=== SensorManager Test ===");

    // Create sensors
    MPU6050Sensor* imu = new MPU6050Sensor(IMU_NAME);
    FSRSensor* fsr = new FSRSensor(FSR_NAME, FSR_PIN);

    // Add sensors to manager
    sensorManager.addSensor(imu);
    sensorManager.addSensor(fsr);

    // Initialize all sensors
    if (sensorManager.initializeAll()) {
        Serial.println("All sensors initialized successfully.");
    } else {
        Serial.println("Sensor initialization failed!");
    }
}

void loop() {
    sensorManager.updateAll();
    sensorManager.printAllSensorData();
    delay(1000); // Print every second
} 