// #include <Arduino.h>
// // #include <memory> // Required for std::unique_ptr and std::make_unique

// // // Include your manager and FSM headers
// // #include "fsm/state_machine.h"
// // #include "sensors/sensor_manager.h"
// // #include "feedback/feedback_manager.h"
// // #include "input/input_manager.h"

// // // Include your CONCRETE implementation headers
// // #include "sensors/force_sensor.h"
// // #include "feedback/vibration_motor.h"

// // #include <memory>
// // #include <utility>

// // // --- Configuration ---
// // // Define the pins for your hardware
// // #define FSR_PIN_FRONT A0
// // #define FSR_PIN_BACK  A1
// // #define MOTOR_PIN_1   13 // The built-in LED pin is great for testing
// // #define MAIN_BUTTON_PIN 2

// // // --- Global Variables ---
// // // The main state machine object. Use a smart pointer for automatic memory management.
// // std::unique_ptr<BeltFSM> beltFsm;

// // void setup() {
// //   Serial.begin(115200);
// //   while (!Serial); // Wait for serial connection
// //   Serial.println("Smart Belt Booting Up...");

// //   // 1. Create the managers
// //   // These will hold and manage the individual components.
// //   auto sensorManager = std::unique_ptr<SensorManager>(new SensorManager());
// //   auto feedbackManager = std::unique_ptr<FeedbackManager>(new FeedbackManager());
  
// //   auto inputManager = std::unique_ptr<InputManager>(new InputManager());
  
// //   // 2. Create and add concrete SENSORS to the SensorManager
// //   // You can add as many sensors as you need.
// //   // 2. Create and add concrete SENSORS to the SensorManager
// //   sensorManager->addSensor(std::unique_ptr<ForceSensor>(new ForceSensor("front_fsr", FSR_PIN_FRONT)));
// //   sensorManager->addSensor(std::unique_ptr<ForceSensor>(new ForceSensor("back_fsr",  FSR_PIN_BACK)));
// //   Serial.println("Sensors created and added.");

// //   // 3. Create and add concrete MOTORS to the FeedbackManager
// //   feedbackManager->addMotor(std::unique_ptr<VibrationMotor>(new VibrationMotor("main_buzzer", MOTOR_PIN_1)));
// //   Serial.println("Feedback motors created and added.");

// //   // 4. Create and add concrete INPUTS to the InputManager
// //   inputManager->addButton("main_button", MAIN_BUTTON_PIN);
// //   Serial.println("Inputs created and added.");

// //   // 5. Create the State Machine (BeltFSM)
// //   //    We still move the managers into the FSM, but build it with new
// //   beltFsm = std::unique_ptr<BeltFSM>(
// //       new BeltFSM(
// //         std::move(sensorManager),
// //         std::move(feedbackManager),
// //         std::move(inputManager)
// //       )
// //   );
// //   Serial.println("BeltFSM created. Initializing...");

// //   Serial.println("BeltFSM created. Initializing...");

// //   // 6. Initialize the FSM
// //   // This will, in turn, call the initializeAll() methods of the managers.
// //   if (!beltFsm->initialize()) {
// //     Serial.println("FATAL: Failed to initialize Belt FSM. System halted.");
// //     // In a real product, you might blink an error LED here.
// //     while (true) {
// //       delay(1000);
// //     }
// //   }

// //   Serial.println("\n--- System Ready! Starting main loop. ---");
// // }

// // void loop() {
// //   // The only thing you need to do in the main loop is update the FSM.
// //   // The FSM will then handle updating all its managed components.
// //   if (beltFsm) {
// //     beltFsm->update();
// //   }
  
// //   // A small delay to prevent the loop from running too fast and starving other tasks.
// //   delay(10); 
// // }

// // // TESTING
// // #include "sensors/imu_sensor.h"
// // #include "sensors/fsr_sensor.h"

// // // Create sensor instances
// // MPU6050Sensor imu("IMU");
// // FSRSensor forceSensor("FSR", A0);  // FSR connected to analog pin A0

// // void setup() {
// //     Serial.begin(115200);
// //     while (!Serial){
// //         delay(10);
// //     }
    
// //     Serial.println("=== Starting sensors ===");
    
// //     // Initialize IMU with detailed feedback
// //     Serial.println("Attempting to initialize IMU...");
// //     bool imuResult = imu.begin();
    
// //     if (imuResult) {
// //         Serial.println("✓ IMU initialized successfully");
// //     } else {
// //         Serial.println("✗ IMU initialization FAILED!");
// //         Serial.println("Check the debug output above for details.");
// //         Serial.println("Halting execution - fix the IMU issue first.");
// //         Serial.println("Common issues:");
// //         Serial.println("  - Check wiring (SDA/SCL/VCC/GND)");
// //         Serial.println("  - Verify pin numbers in config");
// //         Serial.println("  - Ensure 3.3V power supply");
// //         Serial.println("  - Try different I2C pins");
        
// //         // Halt here so you can read the logs
// //         while (true) {
// //             delay(1000);
// //             Serial.println("IMU initialization failed - system halted");
// //         }
// //     }
    
// //     // Only continue if IMU works
// //     Serial.println("Attempting to initialize FSR...");
// //     if (forceSensor.begin()) {
// //         Serial.println("✓ Force sensor initialized successfully");
// //     } else {
// //         Serial.println("✗ Force sensor initialization failed!");
// //         // FSR failure is less critical, continue anyway
// //     }
    
// //     Serial.println("=== Initialization complete ===");
// //     Serial.println("Starting main loop...");
// //     delay(1000);
// // }

// // void loop() {
// //     // Update both sensors
// //     imu.update();
// //     forceSensor.update();
    
// //     // Print IMU data
// //     if (imu.isReady()) {
// //         IMUData imuData = imu.getData();
        
// //         Serial.print("IMU - Accel: (");
// //         Serial.print(imuData.accelX, 2);
// //         Serial.print(", ");
// //         Serial.print(imuData.accelY, 2);
// //         Serial.print(", ");
// //         Serial.print(imuData.accelZ, 2);
// //         Serial.print(") g | Gyro: (");
// //         Serial.print(imuData.gyroX, 1);
// //         Serial.print(", ");
// //         Serial.print(imuData.gyroY, 1);
// //         Serial.print(", ");
// //         Serial.print(imuData.gyroZ, 1);
// //         Serial.print(") °/s | Roll: ");
// //         Serial.print(imu.getRoll(), 1);
// //         Serial.print("° | Pitch: ");
// //         Serial.print(imu.getPitch(), 1);
// //         Serial.print("° | Temp: ");
// //         Serial.print(imuData.temperature, 1);
// //         Serial.print("°C");
// //     } else {
// //         Serial.print("IMU not ready");
// //     }
    
// //     Serial.print(" | ");
    
// //     // Print FSR data
// //     if (forceSensor.isReady()) {
// //         FSRData fsrData = forceSensor.getData();
        
// //         Serial.print("Force: ");
// //         Serial.print(fsrData.force, 3);
// //         Serial.print(" (");
// //         Serial.print(fsrData.rawValue);
// //         Serial.print(")");
        
// //         if (forceSensor.isPressed()) {
// //             Serial.print(" [PRESSED]");
// //         }
// //     } else {
// //         Serial.print("FSR not ready");
// //     }
    
// //     Serial.println();
// //     delay(500); // Slower updates for easier reading
// // }

// #include <Arduino.h>
// #include "sensors/sensor_manager.h"
// #include "sensors/imu_sensor.h"
// #include "sensors/fsr_sensor.h"

// // Pin definitions (adjust as needed)
// #define IMU_NAME "IMU"
// #define FSR_NAME "FSR"
// #define FSR_PIN A0

// SimpleSensorManager sensorManager;

// void setup() {
//     Serial.begin(115200);
//     delay(2000); // Wait for Serial to connect
//     Serial.println("=== SensorManager Test ===");

//     // Create sensors
//     MPU6050Sensor* imu = new MPU6050Sensor(IMU_NAME);
//     FSRSensor* fsr = new FSRSensor(FSR_NAME, FSR_PIN);

//     // Add sensors to manager
//     sensorManager.addSensor(imu);
//     sensorManager.addSensor(fsr);

//     // Initialize all sensors
//     if (sensorManager.initializeAll()) {
//         Serial.println("All sensors initialized successfully.");
//     } else {
//         Serial.println("Sensor initialization failed!");
//     }
// }

// void loop() {
//     sensorManager.updateAll();
//     sensorManager.printAllSensorData();
//     delay(1000); // Print every second
// } 


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

// // TESTING
// #include "sensors/imu_sensor.h"
// #include "sensors/fsr_sensor.h"

// // Create sensor instances
// MPU6050Sensor imu("IMU");
// FSRSensor forceSensor("FSR", A0);  // FSR connected to analog pin A0

// void setup() {
//     Serial.begin(115200);
//     while (!Serial){
//         delay(10);
//     }
    
//     Serial.println("=== Starting sensors ===");
    
//     // Initialize IMU with detailed feedback
//     Serial.println("Attempting to initialize IMU...");
//     bool imuResult = imu.begin();
    
//     if (imuResult) {
//         Serial.println("✓ IMU initialized successfully");
//     } else {
//         Serial.println("✗ IMU initialization FAILED!");
//         Serial.println("Check the debug output above for details.");
//         Serial.println("Halting execution - fix the IMU issue first.");
//         Serial.println("Common issues:");
//         Serial.println("  - Check wiring (SDA/SCL/VCC/GND)");
//         Serial.println("  - Verify pin numbers in config");
//         Serial.println("  - Ensure 3.3V power supply");
//         Serial.println("  - Try different I2C pins");
        
//         // Halt here so you can read the logs
//         while (true) {
//             delay(1000);
//             Serial.println("IMU initialization failed - system halted");
//         }
//     }
    
//     // Only continue if IMU works
//     Serial.println("Attempting to initialize FSR...");
//     if (forceSensor.begin()) {
//         Serial.println("✓ Force sensor initialized successfully");
//     } else {
//         Serial.println("✗ Force sensor initialization failed!");
//         // FSR failure is less critical, continue anyway
//     }
    
//     Serial.println("=== Initialization complete ===");
//     Serial.println("Starting main loop...");
//     delay(1000);
// }

// void loop() {
//     // Update both sensors
//     imu.update();
//     forceSensor.update();
    
//     // Print IMU data
//     if (imu.isReady()) {
//         IMUData imuData = imu.getData();
        
//         Serial.print("IMU - Accel: (");
//         Serial.print(imuData.accelX, 2);
//         Serial.print(", ");
//         Serial.print(imuData.accelY, 2);
//         Serial.print(", ");
//         Serial.print(imuData.accelZ, 2);
//         Serial.print(") g | Gyro: (");
//         Serial.print(imuData.gyroX, 1);
//         Serial.print(", ");
//         Serial.print(imuData.gyroY, 1);
//         Serial.print(", ");
//         Serial.print(imuData.gyroZ, 1);
//         Serial.print(") °/s | Roll: ");
//         Serial.print(imu.getRoll(), 1);
//         Serial.print("° | Pitch: ");
//         Serial.print(imu.getPitch(), 1);
//         Serial.print("° | Temp: ");
//         Serial.print(imuData.temperature, 1);
//         Serial.print("°C");
//     } else {
//         Serial.print("IMU not ready");
//     }
    
//     Serial.print(" | ");
    
//     // Print FSR data
//     if (forceSensor.isReady()) {
//         FSRData fsrData = forceSensor.getData();
        
//         Serial.print("Force: ");
//         Serial.print(fsrData.force, 3);
//         Serial.print(" (");
//         Serial.print(fsrData.rawValue);
//         Serial.print(")");
        
//         if (forceSensor.isPressed()) {
//             Serial.print(" [PRESSED]");
//         }
//     } else {
//         Serial.print("FSR not ready");
//     }
    
//     Serial.println();
//     delay(500); // Slower updates for easier reading
// }

// #include <Arduino.h>
// #include "sensors/sensor_manager.h"
// #include "sensors/imu_sensor.h"
// #include "sensors/fsr_sensor.h"

// // Pin definitions (adjust as needed)
// #define IMU_NAME "IMU"
// #define FSR_NAME "FSR"
// #define FSR_PIN A0

// SimpleSensorManager sensorManager;

// void setup() {
//     Serial.begin(115200);
//     delay(2000); // Wait for Serial to connect
//     Serial.println("=== SensorManager Test ===");

//     // Create sensors
//     MPU6050Sensor* imu = new MPU6050Sensor(IMU_NAME);
//     FSRSensor* fsr = new FSRSensor(FSR_NAME, FSR_PIN);

//     // Add sensors to manager
//     sensorManager.addSensor(imu);
//     sensorManager.addSensor(fsr);

//     // Initialize all sensors
//     if (sensorManager.initializeAll()) {
//         Serial.println("All sensors initialized successfully.");
//     } else {
//         Serial.println("Sensor initialization failed!");
//     }
// }

// void loop() {
//     sensorManager.updateAll();
//     sensorManager.printAllSensorData();
//     delay(1000); // Print every second
// } 

#include <Arduino.h>
#include "feedback/feedback_manager.h"
#include "feedback/vibration_motor.h"
#include "fsm/state_machine.h"
#include "input/input_manager.h"
#include <memory>

// Define the pin the motor is connected to (via the transistor)
// On the Arduino Nano ESP32, pin D5 corresponds to GPIO9.
// const int MOTOR_PIN = 8; 

// Create an instance of our feedback manager
// BeltFSM fsmManager;
FeedbackManager feedbackManager;
InputManager inputManager;

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(115200);
  while (!Serial); // Wait for serial to connect

  Serial.println("--- Arduino Nano ESP32: Feedback Motor Test ---");

  // Create a new vibration motor object using a smart pointer
  // This motor is named "main_vibe" and is connected to MOTOR_PIN (GPIO8 / D5)
  std::unique_ptr<VibrationMotor> myMotor(new VibrationMotor("main_vibe", D5));

  // Add the motor to the manager. std::move transfers ownership.
  feedbackManager.addMotor(std::move(myMotor));

  inputManager.addButton("test_button", D2);
  inputManager.initialize();

  pinMode(D2, INPUT_PULLUP);


  // Initialize all motors managed by the manager
  if (feedbackManager.initializeAll()) {
    Serial.println("Ready to buzz on pin D5!");
  } else {
    Serial.println("Initialization failed. Halting.");
    while(1); // Stop execution
  }
}

void loop() {
  // Buzz the motor(s) every 3 seconds
  inputManager.update();
  delay(100);
  if (inputManager.isButtonPressed("test_button")) {
    Serial.println("Button Pressed");
    feedbackManager.buzzAll();
    inputManager.clearButtonPress("test_button");
    delay(3000);
  } else {
    Serial.println("Button not pressed");
  }

//   bool pressed = (digitalRead(D2) == HIGH);

//   if (pressed) {
//     Serial.println("Pressed");
//   } else {
//     Serial.println("Released");
//   }

//   Serial.println("Buzzing all motors for 150ms...");
//   feedbackManager.buzzAll();
//   Serial.println("Waiting for 0.5 seconds...");
//    delay(1000);
}