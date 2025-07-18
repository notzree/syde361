#include <Arduino.h>

enum class LEDS {
    IDLE,
    CALIBRATING,
    READY
};

class LED {
    public:
        // init
        LED(LEDS type, int pin);
        bool initialize();

        // getters/setters
        void turnOn();
        void turnOff();
        LEDS getType();

        // flash light
        void flash();

    private:
        LEDS type_;
        int pin_;
        bool on_ = false;
};