#include <Arduino.h>
#include "led_interface.h"

LED::LED(LEDS type, int pin) {
    type_ = type;
    pin_ = pin;
}

bool LED::initialize() {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, type_ == LEDS::IDLE ? HIGH : LOW);
}

void LED::turnOn() {
    digitalWrite(pin_, HIGH);
    on_ = true;
    
}

void LED::turnOff() {
    digitalWrite(pin_, LOW);
    on_ = false;
}

void LED::flash() {
    digitalWrite(pin_, HIGH);
    delay(1500);
    digitalWrite(pin_, LOW);
    on_ = false;
}

LEDS LED::getType() {
    return type_;
}