#include <Arduino.h>
// #include <memory> // Required for std::unique_ptr and std::make_unique

// // Include your manager and FSM headers
// #include "fsm/state_machine.h"
// #include "sensors/sensor_manager.h"
// #include "feedback/feedback_manager.h"
// #include "input/input_manager.h"

// // Include your CONCRETE implementation headers
// #include "sensors/force_sensor.h"
// #include "feedback/vibration_motor.h"

// #include <memory>
// #include <utility>

// // --- Configuration ---
// // Define the pins for your hardware
// #define FSR_PIN_FRONT A0
// #define FSR_PIN_BACK  A1
// #define MOTOR_PIN_1   13 // The built-in LED pin is great for testing
// #define MAIN_BUTTON_PIN 2

// // --- Global Variables ---
// // The main state machine object. Use a smart pointer for automatic memory management.
// std::unique_ptr<BeltFSM> beltFsm;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial); // Wait for serial connection
//   Serial.println("Smart Belt Booting Up...");

//   // 1. Create the managers
//   // These will hold and manage the individual components.
//   auto sensorManager = std::unique_ptr<SensorManager>(new SensorManager());
//   auto feedbackManager = std::unique_ptr<FeedbackManager>(new FeedbackManager());
  
//   auto inputManager = std::unique_ptr<InputManager>(new InputManager());
  
//   // 2. Create and add concrete SENSORS to the SensorManager
//   // You can add as many sensors as you need.
//   // 2. Create and add concrete SENSORS to the SensorManager
//   sensorManager->addSensor(std::unique_ptr<ForceSensor>(new ForceSensor("front_fsr", FSR_PIN_FRONT)));
//   sensorManager->addSensor(std::unique_ptr<ForceSensor>(new ForceSensor("back_fsr",  FSR_PIN_BACK)));
//   Serial.println("Sensors created and added.");

//   // 3. Create and add concrete MOTORS to the FeedbackManager
//   feedbackManager->addMotor(std::unique_ptr<VibrationMotor>(new VibrationMotor("main_buzzer", MOTOR_PIN_1)));
//   Serial.println("Feedback motors created and added.");

//   // 4. Create and add concrete INPUTS to the InputManager
//   inputManager->addButton("main_button", MAIN_BUTTON_PIN);
//   Serial.println("Inputs created and added.");

//   // 5. Create the State Machine (BeltFSM)
//   //    We still move the managers into the FSM, but build it with new
//   beltFsm = std::unique_ptr<BeltFSM>(
//       new BeltFSM(
//         std::move(sensorManager),
//         std::move(feedbackManager),
//         std::move(inputManager)
//       )
//   );
//   Serial.println("BeltFSM created. Initializing...");

//   Serial.println("BeltFSM created. Initializing...");

//   // 6. Initialize the FSM
//   // This will, in turn, call the initializeAll() methods of the managers.
//   if (!beltFsm->initialize()) {
//     Serial.println("FATAL: Failed to initialize Belt FSM. System halted.");
//     // In a real product, you might blink an error LED here.
//     while (true) {
//       delay(1000);
//     }
//   }

//   Serial.println("\n--- System Ready! Starting main loop. ---");
// }

// void loop() {
//   // The only thing you need to do in the main loop is update the FSM.
//   // The FSM will then handle updating all its managed components.
//   if (beltFsm) {
//     beltFsm->update();
//   }
  
//   // A small delay to prevent the loop from running too fast and starving other tasks.
//   delay(10); 
// }

// TESTING
#include "sensors/imu_sensor.h"
#include "sensors/fsr_sensor.h"
// Create sensor instances
MPU6050 imu("IMU");
FSRSensor forceSensor("FSR", A0);  // FSR connected to analog pin A0

void setup() {
    Serial.begin(115200);
    Serial.println("Starting sensors...");
    
    // Initialize IMU
    if (imu.begin()) {
        Serial.println("IMU initialized successfully");
    } else {
        Serial.println("IMU initialization failed!");
    }
    
    // Initialize FSR
    if (forceSensor.begin()) {
        Serial.println("Force sensor initialized successfully");
    } else {
        Serial.println("Force sensor initialization failed!");
    }
    
    delay(1000);
}

void loop() {
    // Update both sensors
    imu.update();
    forceSensor.update();
    
    // Print IMU data
    if (imu.isReady()) {
        IMUData imuData = imu.getData();
        
        Serial.print("IMU - Accel: (");
        Serial.print(imuData.accelX, 2);
        Serial.print(", ");
        Serial.print(imuData.accelY, 2);
        Serial.print(", ");
        Serial.print(imuData.accelZ, 2);
        Serial.print(") g | Gyro: (");
        Serial.print(imuData.gyroX, 1);
        Serial.print(", ");
        Serial.print(imuData.gyroY, 1);
        Serial.print(", ");
        Serial.print(imuData.gyroZ, 1);
        Serial.print(") °/s | Roll: ");
        Serial.print(imu.getRoll(), 1);
        Serial.print("° | Pitch: ");
        Serial.print(imu.getPitch(), 1);
        Serial.print("° | Temp: ");
        Serial.print(imuData.temperature, 1);
        Serial.print("°C");
    } else {
        Serial.print("IMU not ready");
    }
    
    Serial.print(" | ");
    
    // Print FSR data
    if (forceSensor.isReady()) {
        FSRData fsrData = forceSensor.getData();
        
        Serial.print("Force: ");
        Serial.print(fsrData.force, 3);
        Serial.print(" (");
        Serial.print(fsrData.rawValue);
        Serial.print(")");
        
        if (forceSensor.isPressed()) {
            Serial.print(" [PRESSED]");
        }
    } else {
        Serial.print("FSR not ready");
    }
    
    Serial.println();
    delay(100);
}