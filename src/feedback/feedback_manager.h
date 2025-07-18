#pragma once
#include "motor_interface.h"
#include <memory>
#include <vector>
#include "led_interface.h"

enum class FeedbackPattern {
    GOOD_BRACE,    // 2 short pulses
    POOR_BRACE,    // 3 long pulses
    CALIBRATION,   // Slow continuous pulse
    ERROR          // Rapid pulses
};

class FeedbackManager {
    public:
        FeedbackManager() = default;
        ~FeedbackManager() = default;

        // Feedback motor management
        void addMotor(std::unique_ptr<IMotor> motor);
        void addLED(std::unique_ptr<LED> led);
        bool initializeAll();
        void update();
        
        void startPattern(FeedbackPattern pattern);
        bool isPatternComplete();
        void stopPattern();
        void updatePattern(); // Call in main loop

        // TODO: turn on/off idle led and move flash into methods
        
    private:
        std::vector<std::unique_ptr<IMotor>> motors_;
        std::unique_ptr<LED> leds_[3];
        unsigned long lastUpdate_;
        FeedbackPattern currentPattern_;
        unsigned long patternStartTime_;
        bool patternActive_;
};