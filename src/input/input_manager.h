#pragma once
#include <Arduino.h>
#include <map>
#include <string>

class InputManager {
public:
    InputManager();
    
    // Add a button with a name and pin
    bool addButton(const std::string& name, int pin);
    
    // Initialize all buttons
    bool initialize();
    
    // Update all buttons
    void update();
    
    // Check if a specific button is pressed
    bool isPressed(const std::string& name);
    
    // Clear the press flag for a specific button
    void clearPress(const std::string& name);

private:
    struct ButtonState {
        int pin;
        bool lastRawState;
        bool lastDebouncedState;
        unsigned long lastDebounceTime;
        bool pressFlag;
        
        ButtonState() : pin(-1), lastRawState(HIGH), lastDebouncedState(HIGH), 
                       lastDebounceTime(0), pressFlag(false) {}
    };
    
    std::map<std::string, ButtonState> buttons_;
    
    static const unsigned long DEBOUNCE_DELAY = 50;
};