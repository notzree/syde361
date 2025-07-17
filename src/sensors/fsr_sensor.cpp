#include "fsm/state_machine.h"
#include "../config/belt_config.h"
#include "fsm/belt_states.h"
#include "sensors/sensor_manager.h"
#include <Arduino.h>
#include <memory>
#include <utility>
#include <iostream>
#include <numeric>
#include <cmath>

BeltFSM::BeltFSM(
    std::unique_ptr<SimpleSensorManager> sensorManager,
    std::unique_ptr<FeedbackManager> feedbackManager,
    std::unique_ptr<InputManager> inputManager
) : currentState_(BeltState::STARTUP),
    stateEntryTime_(0),
    sensorManager_(std::move(sensorManager)),
    inputManager_(std::move(inputManager)),
    feedbackManager_(std::move(feedbackManager)),
    stateChangeCallback_(nullptr),
    lastEventCheck_(0),
    buttonPressTime_(0),
    waitingForSecondPress_(false),
    calibrationStartTime_(0),
    feedbackStartTime_(0)
     {
    baseline_.isCalibrated = false;
}

bool BeltFSM::initialize(){
    Serial.println("attempting to initialize belt fsm...");
    
    if (!sensorManager_->initializeAll()){
        Serial.println("failed to initialize sensors!");
        transitionTo(BeltState::ERROR);
        return false;
    }
    
    if (!inputManager_->initialize()){
        Serial.println("failed to initialize input manager!");
        transitionTo(BeltState::ERROR);
        return false;
    }
    
    if (!feedbackManager_->initializeAll()){
        Serial.println("failed to initialize motors!");
        transitionTo(BeltState::ERROR);
        return false;
    }

    transitionTo(BeltState::STARTUP);
    stateEntryTime_ = millis();
    Serial.println("belt fsm successfully initialized");
    return true;
}

void BeltFSM::update() {
    unsigned long currentTime = millis();
    
    sensorManager_->updateAll();
    inputManager_->update();
    feedbackManager_->update();
    
    if (currentTime - lastEventCheck_ >= BeltConfig::EVENT_CHECK_INTERVAL) {
        BeltEvent event = checkForEvents();
        if (event != BeltEvent::NO_EVENT) {
            handleEvent(event);
        }
        lastEventCheck_ = currentTime;
    }
}

BeltEvent BeltFSM::checkForEvents() {
    unsigned long currentTime = millis();
    
    if (inputManager_->isButtonDoublePressed("main_button")) {
        inputManager_->clearButtonDoublePress("main_button");
        return BeltEvent::BUTTON_DOUBLE_PRESSED;
    }
    
    if (inputManager_->isButtonPressed("main_button")) {
        inputManager_->clearButtonPress("main_button");
        return BeltEvent::BUTTON_PRESSED;
    }
    
    if (!sensorManager_->areAllSensorsHealthy()) {
        return BeltEvent::SENSOR_FAILURE;
    }
    
    switch (currentState_) {
        case BeltState::STARTUP:
            if (sensorManager_->areAllSensorsReady()) {
                return BeltEvent::SENSORS_INITIALIZED;
            }
            break;
            
        case BeltState::CALIBRATION:
            if (currentTime - calibrationStartTime_ >= BeltConfig::CALIBRATION_DURATION) {
                return BeltEvent::CALIBRATION_COMPLETE;
            }
            break;
            
        case BeltState::READY:
            if (isGoodBraceDetected()) {
                return BeltEvent::GOOD_BRACE_DETECTED;
            }
            if (isPoorBraceDetected()) {
                return BeltEvent::POOR_BRACE_DETECTED;
            }
            break;
            
        case BeltState::FEEDBACK_GOOD:
        case BeltState::FEEDBACK_BAD:
            if (feedbackManager_->isPatternComplete()) {
                return BeltEvent::FEEDBACK_COMPLETE;
            }
            break;
        default:
            break;
    }
    
    return BeltEvent::NO_EVENT;
}


void BeltFSM::handleEvent(BeltEvent event) {
    switch (currentState_) {
        case BeltState::STARTUP:
            handleStartupState(event);
            break;
        case BeltState::ERROR:
            handleErrorState(event);
            break;
        case BeltState::IDLE:
            handleIdleState(event);
            break;
        case BeltState::CALIBRATION:
            handleCalibrationState(event);
            break;
        case BeltState::READY:
            handleReadyState(event);
            break;
        case BeltState::FEEDBACK_GOOD:
        case BeltState::FEEDBACK_BAD:
            handleFeedbackState(event);
            break;
    }
}

void BeltFSM::transitionTo(BeltState newState) {
    if (newState == currentState_) return;
    
    BeltState oldState = currentState_;
    
    exitState(currentState_);
    
    currentState_ = newState;
    stateEntryTime_ = millis();
    
    enterState(newState);
    
    if (stateChangeCallback_) {
        stateChangeCallback_(oldState, newState);
    }
}

void BeltFSM::enterState(BeltState state) {
    Serial.print("Entering state: ");
    Serial.println(stateToString(state));
    
    switch (state) {
        case BeltState::CALIBRATION:
            calibrationStartTime_ = millis();
            feedbackManager_->startPattern(FeedbackPattern::CALIBRATION);
            startCalibration();
            break;
            
        case BeltState::FEEDBACK_GOOD:
            feedbackStartTime_ = millis();
            feedbackManager_->startPattern(FeedbackPattern::GOOD_BRACE);
            break;
            
        case BeltState::FEEDBACK_BAD:
            feedbackStartTime_ = millis();
            feedbackManager_->startPattern(FeedbackPattern::POOR_BRACE);
            break;
            
        case BeltState::ERROR:
            feedbackManager_->startPattern(FeedbackPattern::ERROR);
            break;
        default:
            break;
    }
}

void BeltFSM::exitState(BeltState state) {
    Serial.print("Exiting state: ");
    Serial.println(stateToString(state));
    
    switch (state) {
        case BeltState::CALIBRATION:
            baseline_.isCalibrated = true;
            feedbackManager_->stopPattern();
            break;
            
        case BeltState::FEEDBACK_GOOD:
        case BeltState::FEEDBACK_BAD:
            feedbackManager_->stopPattern();
            break;
        default:
            break;
    }
}

void BeltFSM::handleStartupState(BeltEvent event) {
    switch (event) {
        case BeltEvent::SENSORS_INITIALIZED:
            transitionTo(BeltState::IDLE);
            break;
        case BeltEvent::SENSOR_FAILURE:
            transitionTo(BeltState::ERROR);
            break;
        default:
            break;
    }
}

void BeltFSM::handleIdleState(BeltEvent event) {
    switch (event) {
        case BeltEvent::BUTTON_PRESSED:
            transitionTo(BeltState::READY);
            break;
        case BeltEvent::BUTTON_DOUBLE_PRESSED:
            transitionTo(BeltState::CALIBRATION);
            break;
        case BeltEvent::SENSOR_FAILURE:
            transitionTo(BeltState::ERROR);
            break;
        default:
            break;
    }
}

void BeltFSM::handleCalibrationState(BeltEvent event) {
    switch (event) {
        case BeltEvent::CALIBRATION_COMPLETE:
            transitionTo(BeltState::IDLE);
            break;
        case BeltEvent::SENSOR_FAILURE:
        case BeltEvent::CALIBRATION_TIMEOUT:
            transitionTo(BeltState::ERROR);
            break;
        default:
            break;
    }
}

void BeltFSM::handleReadyState(BeltEvent event) {
    switch (event) {
        case BeltEvent::GOOD_BRACE_DETECTED:
            transitionTo(BeltState::FEEDBACK_GOOD);
            break;
        case BeltEvent::POOR_BRACE_DETECTED:
            transitionTo(BeltState::FEEDBACK_BAD);
            break;
        case BeltEvent::BUTTON_PRESSED:
            transitionTo(BeltState::IDLE);
            break;
        case BeltEvent::SENSOR_FAILURE:
            transitionTo(BeltState::ERROR);
            break;
        default:
            break;
    }
}

void BeltFSM::handleFeedbackState(BeltEvent event) {
    if (event == BeltEvent::FEEDBACK_COMPLETE) {
        transitionTo(BeltState::READY);
    }
}

void BeltFSM::handleErrorState(BeltEvent event) {
    if (millis() - stateEntryTime_ > 5000) { // Every 5 seconds
        if (sensorManager_->initializeAll()) {
            transitionTo(BeltState::STARTUP);
        }
    }
}

void BeltFSM::startCalibration() {
    Serial.println("Starting calibration...");
    baseline_.isCalibrated = false;

    // Clear previous baseline data
    for (int i = 0; i < 5; ++i) baseline_.fsrBaselines[i] = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            baseline_.imuBaselines[i][j] = 0;
        }
    }

    // TODO: Add logic to collect and average sensor data over CALIBRATION_DURATION
}

bool BeltFSM::isGoodBraceDetected() const {
    if (!baseline_.isCalibrated) return false;
    
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    if (fsrCount < 2) return false;
    
    float totalWeight = 0;
    float weights[5];
    for (int i = 0; i < fsrCount; i++) {
        FSRData data = fsrSensors[i]->getData();
        weights[i] = data.weight - baseline_.fsrBaselines[i];
        totalWeight += weights[i];
    }
    
    if (totalWeight < BeltConfig::MIN_ENGAGEMENT_WEIGHT) return false;
    
    float variance = calculateVariance(weights, fsrCount);
    bool evenDistribution = variance < BeltConfig::GOOD_DISTRIBUTION_THRESHOLD;
    
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    bool properAlignment = true;
    for (int i = 0; i < imuCount; i++) {
        float pitch = imuSensors[i]->getPitch() - baseline_.imuBaselines[i][1];
        float roll = imuSensors[i]->getRoll() - baseline_.imuBaselines[i][0];
        
        if (abs(pitch) > BeltConfig::PITCH_THRESHOLD || abs(roll) > BeltConfig::ROLL_THRESHOLD) {
            properAlignment = false;
            break;
        }
    }
    
    return evenDistribution && properAlignment;
}

bool BeltFSM::isPoorBraceDetected() const {
    if (!baseline_.isCalibrated) return false;
    
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    float leftSide = 0, rightSide = 0;
    for (int i = 0; i < fsrCount; i++) {
        float weight = fsrSensors[i]->getData().weight - baseline_.fsrBaselines[i];
        if (i % 2 == 0) leftSide += weight;
        else rightSide += weight;
    }
    
    float totalWeight = leftSide + rightSide;
    if (totalWeight < BeltConfig::MIN_ENGAGEMENT_WEIGHT) return false;
    
    float asymmetryRatio = abs(leftSide - rightSide) / totalWeight;
    bool asymmetricPressure = asymmetryRatio > BeltConfig::ASYMMETRY_THRESHOLD;
    
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    bool dangerousAngle = false;
    for (int i = 0; i < imuCount; i++) {
        float pitch = abs(imuSensors[i]->getPitch() - baseline_.imuBaselines[i][1]);
        float roll = abs(imuSensors[i]->getRoll() - baseline_.imuBaselines[i][0]);
        
        if (pitch > BeltConfig::DANGER_PITCH_THRESHOLD || roll > BeltConfig::DANGER_ROLL_THRESHOLD) {
            dangerousAngle = true;
            break;
        }
    }
    
    return asymmetricPressure || dangerousAngle;
}


float BeltFSM::calculateVariance(float* values, int count) const {
    if (count == 0) return 0;
    float sum = 0;
    for (int i = 0; i < count; ++i) {
        sum += values[i];
    }
    float mean = sum / count;
    float variance = 0;
    for (int i = 0; i < count; ++i) {
        variance += std::pow(values[i] - mean, 2);
    }
    return variance / count;
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