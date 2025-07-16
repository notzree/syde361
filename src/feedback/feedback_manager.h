#pragma once
#include "motor_interface.h"
#include <memory>
#include <vector>

class FeedbackManager {
    public:
        FeedbackManager() = default;
        ~FeedbackManager() = default;

        // Feedback motor management
        void addMotor(std::unique_ptr<IMotor> motor);
        bool initializeAll();
        void buzzAll();

    private:
        std::vector<std::unique_ptr<IMotor>> motors_;
        unsigned long lastUpdate_;
};