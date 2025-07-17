#pragma once
#include <Arduino.h>
#include <map>
#include <string>

// A simple struct to manage button state and debouncing
struct Button {
    int pin;
    bool lastState = LOW; // Assuming INPUT_PULLUP
    bool pressed = false;
    bool doublePressed = false;
    unsigned long lastDebounceTime = 0;
    unsigned long lastPressTime = 0;
    int pressCount = 0;
    
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

    bool isButtonDoublePressed(const char* name);
    void clearButtonDoublePress(const char* name);

private:
    std::map<std::string, Button> buttons_;
    static const unsigned long DEBOUNCE_DELAY = 50;
    static const unsigned long DOUBLE_PRESS_WINDOW = 500;
};