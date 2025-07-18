#include "fsm/state_machine.h"
#include "../config/belt_config.h"
#include "USBCDC.h"
#include "fsm/belt_states.h"
#include "sensors/fsr_sensor.h"
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
    feedbackStartTime_(0) {
    baseline_.isCalibrated = false;
    // Initialize calibration tracking
    this->calibrationSampleCount_ = 0;
    this->isCollectingCalibration_ = false;
    for (int i = 0; i < 5; i++) {
        calibrationSums_[i] = 0;
    }
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
    
    if (inputManager_->isPressed("main_button")) {
        inputManager_->clearPress("main_button");
        return BeltEvent::BUTTON_PRESSED;
    }
    
    if (inputManager_->isPressed("calibration_button")) {
        inputManager_->clearPress("calibration_button");
        return BeltEvent::CALIBRATION_BUTTON_PRESSED;
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
                Serial.println("poor bad");
                return BeltEvent::POOR_BRACE_DETECTED;
            }
            break;
            
        case BeltState::FEEDBACK_GOOD:
        case BeltState::FEEDBACK_BAD:
            Serial.println("feedback bad");
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
            // feedbackManager_->startPattern()
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
        case BeltState::IDLE:
            feedbackManager_->startLED(LEDS::IDLE);
            break;
        case BeltState::READY:
            feedbackManager_->startLED(LEDS::READY);
            break;
        case BeltState::CALIBRATION:
            feedbackManager_->startLED(LEDS::CALIBRATING);
            startCalibration();
            feedbackManager_->startPattern(FeedbackPattern::CALIBRATION);
            break;
            
        case BeltState::FEEDBACK_GOOD:
            feedbackStartTime_ = millis();
            feedbackManager_->startPattern(FeedbackPattern::GOOD_BRACE);
            break;
            
        case BeltState::FEEDBACK_BAD:
            feedbackStartTime_ = millis();
            Serial.println("feedback_bad start pattern");
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
        case BeltState::IDLE:
            feedbackManager_->stopLED(LEDS::IDLE);
            break;
        case BeltState::READY:
            feedbackManager_->stopLED(LEDS::READY);
            break;
        case BeltState::CALIBRATION:
            feedbackManager_->stopLED(LEDS::CALIBRATING);
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
            if (baseline_.isCalibrated) {
                transitionTo(BeltState::READY);
            } else {
                Serial.println("Please calibrate first!");
                transitionTo(BeltState::CALIBRATION);
            }
            break;
        case BeltEvent::CALIBRATION_BUTTON_PRESSED:
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
    Serial.println("RELAX YOUR CORE - Stay still for calibration");
    
    baseline_.isCalibrated = false;
    
    // Clear previous baseline data
    for (int i = 0; i < 5; ++i) {
        baseline_.fsrBaselines[i] = 0;
    }
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            baseline_.imuBaselines[i][j] = 0;
        }
    }
    
    // Collect baseline samples
    const int CALIBRATION_SAMPLES = 100;
    const int SAMPLE_DELAY = 50; // ms between samples
    
    Serial.println("Collecting baseline samples...");
    
    // Get FSR sensors
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    // Get IMU sensors
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    // Collect samples
    for (int sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
        // Update sensors
        sensorManager_->updateAll();
        
        // Collect FSR samples
        for (int i = 0; i < fsrCount; i++) {
            FSRData data = fsrSensors[i]->getData();
            baseline_.fsrBaselines[i] += data.rawValue;
        }
        
        // Collect IMU samples
        for (int i = 0; i < imuCount; i++) {
            baseline_.imuBaselines[i][0] += imuSensors[i]->getRoll();
            baseline_.imuBaselines[i][1] += imuSensors[i]->getPitch();
            // baseline_.imuBaselines[i][2] += imuSensors[i]->getYaw();
        }
        
        // Show progress
        if (sample % 10 == 0) {
            Serial.print("Progress: ");
            Serial.print((sample * 100) / CALIBRATION_SAMPLES);
            Serial.println("%");
        }
        
        delay(SAMPLE_DELAY);
    }
    
    // Calculate averages
    for (int i = 0; i < fsrCount; i++) {
        baseline_.fsrBaselines[i] /= CALIBRATION_SAMPLES;
        // Set baseline on each FSR sensor
        fsrSensors[i]->setBaseline((int)baseline_.fsrBaselines[i]);
        
        Serial.print("FSR ");
        Serial.print(i);
        Serial.print(" baseline: ");
        Serial.println(baseline_.fsrBaselines[i]);
    }
    
    for (int i = 0; i < imuCount; i++) {
        baseline_.imuBaselines[i][0] /= CALIBRATION_SAMPLES; // Roll
        baseline_.imuBaselines[i][1] /= CALIBRATION_SAMPLES; // Pitch
        baseline_.imuBaselines[i][2] /= CALIBRATION_SAMPLES; // Yaw
        
        Serial.print("IMU ");
        Serial.print(i);
        Serial.print(" baseline - Roll: ");
        Serial.print(baseline_.imuBaselines[i][0]);
        Serial.print(", Pitch: ");
        Serial.print(baseline_.imuBaselines[i][1]);
        Serial.print(", Yaw: ");
        Serial.println(baseline_.imuBaselines[i][2]);
    }
    
    baseline_.isCalibrated = true;
    Serial.println("Calibration complete! Belt is ready for use.");
}

void BeltFSM::collectCalibrationSample() {
    // Collect samples every CALIBRATION_SAMPLE_INTERVAL milliseconds
    static unsigned long lastSampleTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastSampleTime < BeltConfig::CALIBRATION_SAMPLE_INTERVAL) {
        return;
    }
    
    lastSampleTime = currentTime;
    
    // Collect FSR samples
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    for (int i = 0; i < fsrCount; i++) {
        FSRData data = fsrSensors[i]->getData();
        calibrationSums_[i] += data.rawValue;
    }
    
    // Collect IMU samples
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    for (int i = 0; i < imuCount; i++) {
        baseline_.imuBaselines[i][0] += imuSensors[i]->getRoll();
        baseline_.imuBaselines[i][1] += imuSensors[i]->getPitch();
        // baseline_.imuBaselines[i][2] += imuSensors[i]->getYaw();
    }
    
    calibrationSampleCount_++;
    
    // Show progress
    if (calibrationSampleCount_ % 10 == 0) {
        Serial.print("Calibration progress: ");
        Serial.print((calibrationSampleCount_ * 100) / BeltConfig::CALIBRATION_SAMPLE_COUNT);
        Serial.println("%");
    }
}

void BeltFSM::finishCalibration() {
    if (calibrationSampleCount_ == 0) {
        Serial.println("Calibration failed - no samples collected!");
        baseline_.isCalibrated = false;
        return;
    }
    
    isCollectingCalibration_ = false;
    
    // Calculate averages for FSR sensors
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    for (int i = 0; i < fsrCount; i++) {
        baseline_.fsrBaselines[i] = calibrationSums_[i] / calibrationSampleCount_;
        // Set the baseline on each FSR sensor
        fsrSensors[i]->setBaseline(baseline_.fsrBaselines[i]);
        
        Serial.print("FSR ");
        Serial.print(i);
        Serial.print(" baseline: ");
        Serial.println(baseline_.fsrBaselines[i]);
    }
    
    // Calculate averages for IMU sensors
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    for (int i = 0; i < imuCount; i++) {
        baseline_.imuBaselines[i][0] /= calibrationSampleCount_; // Roll
        baseline_.imuBaselines[i][1] /= calibrationSampleCount_; // Pitch
        baseline_.imuBaselines[i][2] /= calibrationSampleCount_; // Yaw
        
        Serial.print("IMU ");
        Serial.print(i);
        Serial.print(" baseline - Roll: ");
        Serial.print(baseline_.imuBaselines[i][0]);
        Serial.print(", Pitch: ");
        Serial.print(baseline_.imuBaselines[i][1]);
        Serial.print(", Yaw: ");
        Serial.println(baseline_.imuBaselines[i][2]);
    }
    
    baseline_.isCalibrated = true;
    Serial.println("Calibration complete! Belt is ready for use.");
}

bool BeltFSM::isGoodBraceDetected() const {
    if (!baseline_.isCalibrated) return false;
    
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    float totalPressure = 0;
    float pressures[5];
    Serial.println("sensor count:");
    Serial.println(fsrCount);
    for (int i = 0; i < fsrCount; i++) {
        FSRData data = fsrSensors[i]->getData();
        // data.weight now contains pressure above baseline
        pressures[i] = data.weight;
        totalPressure += pressures[i];
        
        Serial.print("FSR ");
        Serial.print(i);
        Serial.print(" pressure above baseline: ");
        Serial.println(pressures[i]);
    }
    
    Serial.print("Total pressure above baseline: ");
    Serial.println(totalPressure);
    
    // Check if enough pressure is being applied
    if (totalPressure < BeltConfig::MIN_ENGAGEMENT_PRESSURE) {
        return false;
    }
    
    // Check for even distribution of pressure
    float variance = calculateVariance(pressures, fsrCount);
    bool evenDistribution = variance < BeltConfig::GOOD_DISTRIBUTION_THRESHOLD;
    
    Serial.print("Pressure variance: ");
    Serial.print(variance);
    Serial.print(", Even distribution: ");
    Serial.println(evenDistribution ? "YES" : "NO");
    
    // Check IMU alignment
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    bool properAlignment = true;
    for (int i = 0; i < imuCount; i++) {
        float pitch = imuSensors[i]->getPitch() - baseline_.imuBaselines[i][1];
        float roll = imuSensors[i]->getRoll() - baseline_.imuBaselines[i][0];
        
        if (abs(pitch) > BeltConfig::PITCH_THRESHOLD || abs(roll) > BeltConfig::ROLL_THRESHOLD) {
            properAlignment = false;
            Serial.print("IMU ");
            Serial.print(i);
            Serial.print(" out of alignment - Pitch: ");
            Serial.print(pitch);
            Serial.print(", Roll: ");
            Serial.println(roll);
            break;
        }
    }
    
    bool goodBrace = evenDistribution && properAlignment;
    if (goodBrace) {
        Serial.println("GOOD BRACE DETECTED!");
    }
    
    return goodBrace;
}

bool BeltFSM::isPoorBraceDetected() const {
    if (!baseline_.isCalibrated) return false;
    
    FSRSensor* fsrSensors[5];
    int fsrCount = sensorManager_->getAllFSRSensors(fsrSensors, 5);
    
    float leftSide = 0, rightSide = 0;
    for (int i = 0; i < fsrCount; i++) {
        FSRData data = fsrSensors[i]->getData();
        float pressure = data.weight; // weight field contains pressure above baseline
        if (i % 2 == 0) leftSide += pressure;
        else rightSide += pressure;
    }
    
    float totalPressure = leftSide + rightSide;
    if (totalPressure < BeltConfig::MIN_ENGAGEMENT_PRESSURE) return false;
    
    // Check for asymmetric pressure distribution
    float asymmetryRatio = abs(leftSide - rightSide) / totalPressure;
    bool asymmetricPressure = asymmetryRatio > BeltConfig::ASYMMETRY_THRESHOLD;
    
    if (asymmetricPressure) {
        Serial.print("Asymmetric pressure detected - Left: ");
        Serial.print(leftSide);
        Serial.print(", Right: ");
        Serial.print(rightSide);
        Serial.print(", Ratio: ");
        Serial.println(asymmetryRatio);
    }
    
    // Check for dangerous angles
    MPU6050Sensor* imuSensors[3];
    int imuCount = sensorManager_->getAllIMUSensors(imuSensors, 3);
    
    bool dangerousAngle = false;
    for (int i = 0; i < imuCount; i++) {
        float pitch = abs(imuSensors[i]->getPitch() - baseline_.imuBaselines[i][1]);
        float roll = abs(imuSensors[i]->getRoll() - baseline_.imuBaselines[i][0]);
        
        if (pitch > BeltConfig::DANGER_PITCH_THRESHOLD || roll > BeltConfig::DANGER_ROLL_THRESHOLD) {
            dangerousAngle = true;
            Serial.print("Dangerous angle detected on IMU ");
            Serial.print(i);
            Serial.print(" - Pitch: ");
            Serial.print(pitch);
            Serial.print(", Roll: ");
            Serial.println(roll);
            break;
        }
    }
    
    bool poorBrace = asymmetricPressure || dangerousAngle;
    if (poorBrace) {
        Serial.println("POOR BRACE DETECTED!");
    }
    
    return poorBrace;
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