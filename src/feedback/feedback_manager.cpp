#include "feedback_manager.h"
#include <Arduino.h>

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

void FeedbackManager::buzzAll() {
    for (auto& motor : motors_) {
        motor->buzz();
    }
} 