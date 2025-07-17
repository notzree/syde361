#include "input_manager.h"
#include "Arduino.h"

InputManager::InputManager() = default;

void InputManager::addButton(const char* name, int pin) {
    buttons_[name] = Button{pin};
}

bool InputManager::initialize() {
    for (auto const& [name, button] : buttons_) {
        pinMode(button.pin, INPUT_PULLUP);
    }
    return true;
}

void InputManager::update() {
    unsigned long currentTime = millis();
    
    for (auto& [name, button] : buttons_) {
        bool reading = digitalRead(button.pin) == LOW; // Active low with pullup
        
        // Debouncing
        if (reading != button.lastState) {
            button.lastDebounceTime = currentTime;
        }
        
        if ((currentTime - button.lastDebounceTime) > DEBOUNCE_DELAY) {
            // Rising edge detection (button press)
            if (reading && !button.lastState) {
                button.pressCount++;
                button.lastPressTime = currentTime;
                
                if (button.pressCount == 1) {
                    // First press
                    button.pressed = true;
                } else if (button.pressCount == 2 && 
                          (currentTime - button.lastPressTime) < DOUBLE_PRESS_WINDOW) {
                    // Second press within window
                    button.doublePressed = true;
                    button.pressed = false; // Clear single press
                }
            }
        }
        
        // Reset press count after window expires
        if (button.pressCount > 0 && 
            (currentTime - button.lastPressTime) > DOUBLE_PRESS_WINDOW) {
            button.pressCount = 0;
        }
        
        button.lastState = reading;
    }
}


bool InputManager::isButtonPressed(const char* name) {
    if (buttons_.count(name)) {
        return buttons_.at(name).pressed;
    }
    return false;
}

void InputManager::clearButtonPress(const char* name) {
    if (buttons_.count(name)) {
        buttons_.at(name).pressed = false;
    }
}

bool InputManager::isButtonDoublePressed(const char* name) {
    if (buttons_.count(name)) {
        return buttons_.at(name).doublePressed;
    }
    return false;
}

void InputManager::clearButtonDoublePress(const char* name) {
    if (buttons_.count(name)) {
        buttons_.at(name).doublePressed = false;
    }
}