#pragma once
#include <Arduino.h>
#include <map>
#include <string>

// A simple struct to manage button state and debouncing
struct Button {
    int pin;
    bool lastState = HIGH; // Assuming INPUT_PULLUP
    bool pressed = false;
    unsigned long lastDebounceTime = 0;
    
    // Constructor that accepts a pin
    Button(int p) : pin(p) {}
    
    // Default constructor
    Button() : pin(-1) {}
};

class InputManager {
public:
    InputManager();
    
    void addButton(const char* name, int pin);
    bool initialize();
    void update();
    
    bool isButtonPressed(const char* name);
    void clearButtonPress(const char* name);

private:
    std::map<std::string, Button> buttons_;
    unsigned long debounceDelay_ = 50; // 50ms debounce delay
};