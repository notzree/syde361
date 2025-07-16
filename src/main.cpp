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

#include <Arduino.h>
#include "sensors/sensor_manager.h"
#include "sensors/imu_sensor.h"
#include "sensors/fsr_sensor.h"
#include "feedback/feedback_manager.h"
#include "feedback/vibration_motor.h"
#include "fsm/state_machine.h"
#include <memory>
#include <utility>

// Pin definitions (adjust as needed)
#define IMU_NAME "IMU"
#define FSR_NAME "FSR"
#define VIBRATION_MOTOR_NAME "VIBRATION MOTOR"
#define FSR_PIN A0
#define VIBRATION_MOTOR_PIN 4

std::unique_ptr<BeltFSM> beltFsm;

// SimpleSensorManager sensorManager;
// FeedbackManager feedbackManager;

const int powerPin = 4;

void setup() {
    Serial.begin(115200);
    Serial.println("=== Starting ===");

    delay(2000); // Wait for Serial to connect
    Serial.println("=== SensorManager Test ===");

    // power pin setup
    pinMode(powerPin, OUTPUT);
    digitalWrite(powerPin, LOW);  // ensure it starts OFF (optional)

    // managers
    auto sensorManager = std::unique_ptr<SimpleSensorManager>(new SimpleSensorManager());
    auto feedbackManager = std::unique_ptr<FeedbackManager>(new FeedbackManager());
    auto inputManager = std::unique_ptr<InputManager>(new InputManager());

    // Create sensors
    MPU6050Sensor* imu = new MPU6050Sensor(IMU_NAME);
    FSRSensor* fsr = new FSRSensor(FSR_NAME, FSR_PIN);
    VibrationMotor* vibrationMotor = new VibrationMotor(VIBRATION_MOTOR_NAME, VIBRATION_MOTOR_PIN);
    
    // Add sensors to manager
    sensorManager->addSensor(imu);
    sensorManager->addSensor(fsr);

    beltFsm = std::unique_ptr<BeltFSM>(
        new BeltFSM(
            std::move(sensorManager),
            std::move(feedbackManager),
            std::move(inputManager)
        )
    );

    // Initialize all sensors
    if (sensorManager->initializeAll()) {
        Serial.println("All sensors initialized successfully.");
    } else {
        Serial.println("Sensor initialization failed!");
    }

}

void loop() {
    // 1) update and print your sensors
    // sensorManager->updateAll();
    // sensorManager->printAllSensorData();
    // 2) turn ON D4
    // digitalWrite(powerPin, HIGH);

    // //if you want to blink instead of constant ON, uncomment:
    // delay(1000);
    // digitalWrite(powerPin, LOW);
    // delay(1000);

    // if (beltFsm) {
    //     beltFsm->update();
    // }
    // beltFsm->forceTransition(BeltState::FEEDBACK_POOR);
    delay(1000);
}
