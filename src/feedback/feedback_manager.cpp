#include "feedback_manager.h"
#include <Arduino.h>
#include "../config/belt_config.h"

void FeedbackManager::addMotor(std::unique_ptr<IMotor> motor) {
    motors_.push_back(std::move(motor));
}

bool FeedbackManager::initializeAll() {
    Serial.println("Initializing all motors...");
    for (auto& motor : motors_) {
        if (!motor->initialize()) {
            Serial.println("Failed to initialize motor");
            return false;
        }
    }
    Serial.println("All motors initialized successfully");
    return true;
}

void FeedbackManager::update() {
    if (patternActive_) {
        updatePattern();
    }
}

void FeedbackManager::startPattern(FeedbackPattern pattern) {
    currentPattern_ = pattern;
    patternStartTime_ = millis();
    patternActive_ = true;
}

bool FeedbackManager::isPatternComplete() {
    if (!patternActive_) {
        return true;
    }

    unsigned long currentTime = millis();
    unsigned long duration = 0;

    switch (currentPattern_) {
        case FeedbackPattern::GOOD_BRACE:
            duration = 400; // 2 short pulses (150ms on, 100ms gap) = 400ms
            break;
        case FeedbackPattern::POOR_BRACE:
            duration = 1050; // 3 long pulses (250ms on, 150ms gap) = 1050ms
            break;
        case FeedbackPattern::CALIBRATION:
            // Calibration runs for a fixed duration, so the pattern is controlled externally
            return false;
        case FeedbackPattern::ERROR:
            duration = 900; // 5 rapid pulses (100ms on, 100ms off) = 900ms
            break;
    }

    if (currentTime - patternStartTime_ >= duration) {
        patternActive_ = false;
        for (auto& motor : motors_) {
            motor->stop();
        }
        return true;
    }

    return false;
}

void FeedbackManager::stopPattern() {
    patternActive_ = false;
    for (auto& motor : motors_) {
        motor->stop();
    }
}

void FeedbackManager::updatePattern() {
    if (!patternActive_) {
        return;
    }

    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - patternStartTime_;

    switch (currentPattern_) {
        case FeedbackPattern::GOOD_BRACE:
            if (elapsedTime < 150) { // Pulse 1
                for (auto& motor : motors_) motor->buzz();
            } else if (elapsedTime < 250) { // Gap 1
                for (auto& motor : motors_) motor->stop();
            } else if (elapsedTime < 400) { // Pulse 2
                for (auto& motor : motors_) motor->buzz();
            } else {
                stopPattern();
            }
            break;

        case FeedbackPattern::POOR_BRACE:
            if (elapsedTime < 250) { // Pulse 1
                for (auto& motor : motors_) motor->buzz();
            } else if (elapsedTime < 400) { // Gap 1
                for (auto& motor : motors_) motor->stop();
            } else if (elapsedTime < 650) { // Pulse 2
                for (auto& motor : motors_) motor->buzz();
            } else if (elapsedTime < 800) { // Gap 2
                for (auto& motor : motors_) motor->stop();
            } else if (elapsedTime < 1050) { // Pulse 3
                for (auto& motor : motors_) motor->buzz();
            } else {
                stopPattern();
            }
            break;

        case FeedbackPattern::CALIBRATION:
            if ((elapsedTime / 500) % 2 == 0) { // 500ms on, 500ms off
                for (auto& motor : motors_) motor->buzz();
            } else {
                for (auto& motor : motors_) motor->stop();
            }
            break;

        case FeedbackPattern::ERROR:
            if ((elapsedTime / 100) % 2 == 0 && elapsedTime < 1000) { // 100ms on, 100ms off, 5 times
                for (auto& motor : motors_) motor->buzz();
            } else {
                for (auto& motor : motors_) motor->stop();
            }
            break;
    }
} 