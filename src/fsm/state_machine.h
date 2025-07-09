#pragma once
#include "belt_states.h"
#include "belt_events.h"
#include "../sensors/sensor_manager.h"
#include "../input/input_manager.h"  // Add this include
#include "../feedback/feedback_manager.h"
#include <memory>
#include <functional>
#include <vector>

class BeltFSM {
public:
    using StateChangeCallback = std::function<void(BeltState, BeltState)>;
    
    explicit BeltFSM(
        std::unique_ptr<SensorManager> sensorManager,
        std::unique_ptr<FeedbackManager> feedbackManager,
        std::unique_ptr<InputManager> inputManager  // Add this parameter
    );
    
    ~BeltFSM() = default;
    
    // Core FSM operations
    bool initialize();
    void update();
    void handleEvent(BeltEvent event);
    
    // State queries
    BeltState getCurrentState() const { return currentState_; }
    unsigned long getStateTime() const;
    
    // Callbacks
    void setStateChangeCallback(StateChangeCallback callback);
    
    // Manual transitions (for testing/debugging)
    void forceTransition(BeltState newState);
    
private:
    // State management
    void transitionTo(BeltState newState);
    void enterState(BeltState state);
    void exitState(BeltState state);
    
    // Event generation
    BeltEvent checkForEvents();
    
    // State handlers
    void handleStartupState(BeltEvent event);
    void handleCalibrationState(BeltEvent event);
    void handleReadyState(BeltEvent event);
    void handleFeedbackGoodState(BeltEvent event);
    void handleFeedbackPoorState(BeltEvent event);
    void handleErrorState(BeltEvent event);
    
    // Analysis methods
    bool isGoodBraceDetected() const;
    bool isPoorBraceDetected() const;
    bool isLiftComplete() const;
    
    // Calibration methods
    void startCalibration();
    void isCalibrated();
    
    // Member variables
    BeltState currentState_;
    unsigned long stateEntryTime_;
    unsigned long lastActivity_;
    
    // Components (injected dependencies)
    std::unique_ptr<SensorManager> sensorManager_;
    std::unique_ptr<InputManager> inputManager_;  // Add this member
    std::unique_ptr<FeedbackManager> feedbackManager_;
    
    // Callbacks
    StateChangeCallback stateChangeCallback_;
    
    // Calibration data
    std::vector<float> baselineData_;
    std::vector<float> braceData_;
    float braceThreshold_;
    
    // Timing
    unsigned long lastEventCheck_;
};