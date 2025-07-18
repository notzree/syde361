#include <Arduino.h>
#include <memory>
#include <utility>

#include "fsm/state_machine.h"
#include "sensors/sensor_manager.h"
#include "feedback/feedback_manager.h"
#include "input/input_manager.h"
#include "sensors/imu_sensor.h"
#include "sensors/fsr_sensor.h"
#include "feedback/vibration_motor.h"


// --- Configuration ---
#define FSR_PIN_1 A0
// #define FSR_PIN_2 A1
// #define FSR_PIN_3 A2
// #define FSR_PIN_4 A3
// #define FSR_PIN_5 A4

#define MOTOR_PIN_1 D6

#define MAIN_BUTTON_PIN D2
#define CALIBRATION_BUTTON_PIN D3


// -- IMPORTANT!!! Change to whichever pin the leds are actually in --
#define IDLE_LED_PIN D7
#define READY_LED_PIN D8
#define CALIBRATION_LED_PIN D9

// --- Global Variables ---
std::unique_ptr<BeltFSM> beltFsm;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Smart Belt Booting Up...");

  auto sensorManager = std::unique_ptr<SimpleSensorManager>(new SimpleSensorManager());
  auto feedbackManager = std::unique_ptr<FeedbackManager>(new FeedbackManager());
  auto inputManager = std::unique_ptr<InputManager>(new InputManager());

  auto fsr1 = std::unique_ptr<FSRSensor>(new FSRSensor("fsr1", FSR_PIN_1));
  sensorManager->addSensor(fsr1.release());

  auto imu1 = std::unique_ptr<MPU6050Sensor>(new MPU6050Sensor("imu1"));
  sensorManager->addSensor(imu1.release());
  
  auto motor1 = std::unique_ptr<VibrationMotor>(new VibrationMotor("motor1", MOTOR_PIN_1));
  feedbackManager->addMotor(std::move(motor1));

  auto idleLED = std::unique_ptr<LED>(new LED(LEDS::IDLE, IDLE_LED_PIN));
  feedbackManager->addLED(std::move(idleLED));

  auto readyLED = std::unique_ptr<LED>(new LED(LEDS::READY, READY_LED_PIN));
  feedbackManager->addLED(std::move(readyLED));

  auto calibrationLED = std::unique_ptr<LED>(new LED(LEDS::CALIBRATING, CALIBRATION_LED_PIN));
  feedbackManager->addLED(std::move(calibrationLED));

  inputManager->addButton("main_button", MAIN_BUTTON_PIN);
  inputManager->addButton("calibration_button", CALIBRATION_BUTTON_PIN);
  
  beltFsm = std::unique_ptr<BeltFSM>(new BeltFSM(
      std::move(sensorManager),
      std::move(feedbackManager),
      std::move(inputManager)
  ));

 

  if (!beltFsm->initialize()) {
    Serial.println("FATAL: Failed to initialize Belt FSM. System halted.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("\n--- System Ready! Starting main loop. ---");
}

void loop() {
  if (beltFsm) {
    beltFsm->update();
  }
  delay(10); 
}