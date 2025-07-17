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
        bool reading = digitalRead(button.pin);

        // If the switch changed, due to noise or pressing
        if (reading != button.lastState) {
            button.lastDebounceTime = currentTime;
        }
  
        Serial.println("Debounce Delay");
        Serial.println(button.lastState);
        Serial.println(reading);

        if (reading == HIGH && button.lastState == LOW) {
            button.lastState = HIGH;
        } else if (reading == HIGH && button.lastState == HIGH) {
            button.lastState = LOW;
        }

        // if ((currentTime - button.lastDebounceTime) > debounceDelay_) {
            

        //     // A press is a transition from HIGH to LOW
        //     if (reading == HIGH && button.lastState == LOW) {
        //         button.pressed = true;
        //     }
        // }
        // button.lastState = reading;
    }
}

bool InputManager::isButtonPressed(const char* name) {
    if (buttons_.count(name)) {
        return buttons_.at(name).lastState;
    }
    return false;
}

void InputManager::clearButtonPress(const char* name) {
    if (buttons_.count(name)) {
        buttons_.at(name).lastState = LOW;
    }
}