#include "fsm/state_machine.h"
#include "../config/belt_config.h"
#include "esp32-hal.h"
#include "fsm/belt_states.h"
#include "sensors/sensor_manager.h"
#include <Arduino.h>
#include <memory>
#include <utility>

BeltFSM::BeltFSM(
    std::unique_ptr<SimpleSensorManager> sensorManager,
    std::unique_ptr<FeedbackManager> feedbackManager,
    std::unique_ptr<InputManager> inputManager
): currentState_(BeltState::STARTUP),
   stateEntryTime_(0),
   lastActivity_(0),
   sensorManager_(std::move(sensorManager)),
   inputManager_(std::move(inputManager)),
   feedbackManager_(std::move(feedbackManager)),
   stateChangeCallback_(nullptr),
   baselineData_(),
   braceData_(),
   braceThreshold_(BeltConfig::BRACE_THRESHOLD),
   lastEventCheck_(0) {
}

bool BeltFSM::initialize(){
    Serial.println("attempting to initialize belt fsm...");
    
    if (!sensorManager_->initializeAll()){
        Serial.println("failed to initialize sensors!");
        return false;
    }
    
    if (!inputManager_->initialize()){
        Serial.println("failed to initialize input manager!");
        return false;
    }
    
    if (!feedbackManager_->initializeAll()){
        Serial.println("failed to initialize motors!");
        return false;
    }

    //setup callbacks
    sensorManager_->setEventCallback([this](const char* sensor, const SensorData& data) {
        // Handle sensor events
        this->lastActivity_ = millis();
    });
    
    currentState_ = BeltState::STARTUP;
    stateEntryTime_ = millis();
    Serial.println("belt fsm successfully initialized");
    return true;
}

void BeltFSM::transitionTo(BeltState newState) {
    if (newState == currentState_) return;
    
    BeltState oldState = currentState_;
    
    // Notify callback if set
    if (stateChangeCallback_) {
        stateChangeCallback_(oldState, newState);
    }
    
    Serial.print("FSM: ");
    Serial.print(stateToString(oldState));
    Serial.print(" -> ");
    Serial.println(stateToString(newState));
    
    // Exit current state
    exitState(currentState_);
    
    // Update state
    currentState_ = newState;
    stateEntryTime_ = millis();
    
    // Enter new state
    enterState(newState);
}

BeltEvent BeltFSM::checkForEvents() {
    // Check for button events first (highest priority)
    if (inputManager_->isButtonPressed("main_button")) {
        inputManager_->clearButtonPress("main_button");
        return BeltEvent::BUTTON_PRESSED;
    }
    
    // Check if sensors are healthy
    if (!sensorManager_->areAllSensorsHealthy()) {
        return BeltEvent::SENSOR_FAILURE;
    }
    
    // State-specific event checking
    switch (currentState_) {
        case BeltState::STARTUP:
            // sensors must be all initialized if we reach here
            return BeltEvent::SENSORS_INITIALIZED;
            
        case BeltState::READY:
            if (isGoodBraceDetected()) {
                return BeltEvent::GOOD_BRACE_DETECTED;
            }
            if (isPoorBraceDetected()) {
                return BeltEvent::POOR_BRACE_DETECTED;
            }
            if (isLiftComplete()) {
                return BeltEvent::FEEDBACK_COMPLETE;
            }
            break;
            
        default:
            break;
    }
    
    return static_cast<BeltEvent>(-1); // No event
}

void BeltFSM::update() {
    unsigned long currentTime = millis();
    
    // Update subsystems
    sensorManager_->updateAll();
    inputManager_->update();
    
    // Check for events periodically
    if (currentTime - lastEventCheck_ >= 50) { // 20Hz event checking
        BeltEvent event = checkForEvents();
        if (event != static_cast<BeltEvent>(-1)) {
            handleEvent(event);
        }
        lastEventCheck_ = currentTime;
    }
    
    // // Handle timeout events
    // checkTimeouts(currentTime);
}

bool BeltFSM::isLiftComplete() const {
    // For now, return false as a placeholder
    // This should be implemented based on your specific lift completion logic
    return false;
}

bool BeltFSM::isGoodBraceDetected() const {
    // For now, return false as a placeholder
    // This should be implemented based on your sensor data analysis
    return false;
}

bool BeltFSM::isPoorBraceDetected() const {
    // For now, return false as a placeholder
    // This should be implemented based on your sensor data analysis
    return false;
}

void BeltFSM::enterState(BeltState state) {
    // State entry logic can be added here
    Serial.print("Entering state: ");
    Serial.println(stateToString(state));
}

void BeltFSM::exitState(BeltState state) {
    // State exit logic can be added here
    Serial.print("Exiting state: ");
    Serial.println(stateToString(state));
}

void BeltFSM::handleEvent(BeltEvent event) {
    // Handle events based on current state
    switch (currentState_) {
        case BeltState::STARTUP:
            handleStartupState(event);
            break;
        case BeltState::READY:
            handleReadyState(event);
            break;
        default:
            // Handle other states as needed
            break;
    }
}

void BeltFSM::handleStartupState(BeltEvent event) {
    if (event == BeltEvent::SENSORS_INITIALIZED) {
        transitionTo(BeltState::READY);
    }
}

void BeltFSM::handleReadyState(BeltEvent event) {
    switch (event) {
        case BeltEvent::GOOD_BRACE_DETECTED:
            // Handle good brace detection
            break;
        case BeltEvent::POOR_BRACE_DETECTED:
            // Handle poor brace detection
            break;
        case BeltEvent::BUTTON_PRESSED:
            // Handle button press
            break;
        default:
            break;
    }
}

void BeltFSM::startCalibration() {
    // Implementation for starting calibration
    Serial.println("Starting calibration...");
}

void BeltFSM::isCalibrated() {
    // Implementation for checking if calibrated
    Serial.println("Checking calibration status...");
}

unsigned long BeltFSM::getStateTime() const {
    return millis() - stateEntryTime_;
}

void BeltFSM::setStateChangeCallback(StateChangeCallback callback) {
    stateChangeCallback_ = callback;
}

void BeltFSM::forceTransition(BeltState newState) {
    transitionTo(newState);
}