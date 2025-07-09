#include "state_machine.h"
#include "../config/belt_config.h"
#include "esp32-hal.h"
#include "fsm/belt_states.h"
#include <Arduino.h>

BeltFSM::BeltFSM(
    std::unique_ptr<SensorManager> sensorManager,
    std::unique_ptr<FeedbackManager> feedbackManager
): currentState_(BeltState::STARTUP),
   stateEntryTime_(0),
   lastActivity_(0),
   sensorManager_(std::move(sensorManager)),
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
    if (!sensorManager_->initializeAll()){
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
            if (isLiftStarted()) {
                return BeltEvent::LIFT_STARTED;
            }
            break;
            
        case BeltState::MONITORING:
            if (isGoodBraceDetected()) {
                return BeltEvent::GOOD_BRACE_DETECTED;
            }
            if (isPoorBraceDetected()) {
                return BeltEvent::POOR_BRACE_DETECTED;
            }
            if (isLiftComplete()) {
                return BeltEvent::LIFT_COMPLETE;
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
    // connectivityManager_->update();
    
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