#include "input_manager.h"
#include "Arduino.h"

InputManager::InputManager() {}

bool InputManager::addButton(const std::string& name, int pin) {
    // Check if button already exists
    if (buttons_.find(name) != buttons_.end()) {
        return false; // Button with this name already exists
    }
    
    // Create new button state
    ButtonState newButton;
    newButton.pin = pin;
    buttons_[name] = newButton;
    
    return true;
}

bool InputManager::initialize() {
    if (buttons_.empty()) return false;
    
    // Initialize all buttons
    for (auto& pair : buttons_) {
        ButtonState& button = pair.second;
        
        if (button.pin == -1) continue;
        
        pinMode(button.pin, INPUT_PULLUP);
        button.lastRawState = digitalRead(button.pin);
        button.lastDebouncedState = button.lastRawState;
    }
    
    return true;
}

void InputManager::update() {
    unsigned long currentTime = millis();
    
    // Update all buttons
    for (auto& pair : buttons_) {
        ButtonState& button = pair.second;
        
        if (button.pin == -1) continue;
        
        bool currentRawState = digitalRead(button.pin);
        
        // Check if raw state changed (start debounce timer)
        if (currentRawState != button.lastRawState) {
            button.lastDebounceTime = currentTime;
            button.lastRawState = currentRawState;
        }
        
        // Check if enough time has passed for debouncing
        if ((currentTime - button.lastDebounceTime) > DEBOUNCE_DELAY) {
            bool currentDebouncedState = currentRawState;
            
            // Detect press event (HIGH to LOW transition with pullup)
            if (button.lastDebouncedState == HIGH && currentDebouncedState == LOW) {
                button.pressFlag = true;
            }
            
            button.lastDebouncedState = currentDebouncedState;
        }
    }
}

bool InputManager::isPressed(const std::string& name) {
    auto it = buttons_.find(name);
    if (it != buttons_.end()) {
        return it->second.pressFlag;
    }
    return false;
}

void InputManager::clearPress(const std::string& name) {
    auto it = buttons_.find(name);
    if (it != buttons_.end()) {
        it->second.pressFlag = false;
    }
}