#pragma once
#include "motor_interface.h"
#include <memory>
#include <vector>

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
        bool initializeAll();
        void update();
        
        void startPattern(FeedbackPattern pattern);
        bool isPatternComplete();
        void stopPattern();
        void updatePattern(); // Call in main loop

    private:
        std::vector<std::unique_ptr<IMotor>> motors_;
        unsigned long lastUpdate_;
        FeedbackPattern currentPattern_;
        unsigned long patternStartTime_;
        bool patternActive_;
};