#pragma once
#include "belt_states.h"
#include "belt_events.h"
#include "../sensors/sensor_manager.h"
#include "../input/input_manager.h"
#include "../feedback/feedback_manager.h"
#include <memory>
#include <functional>
#include <vector>

class BeltFSM {
public:
    using StateChangeCallback = std::function<void(BeltState, BeltState)>;
    
    explicit BeltFSM(
        std::unique_ptr<SimpleSensorManager> sensorManager,
        std::unique_ptr<FeedbackManager> feedbackManager,
        std::unique_ptr<InputManager> inputManager
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
    void handleIdleState(BeltEvent event);
    void handleCalibrationState(BeltEvent event);
    void handleReadyState(BeltEvent event);
    void handleFeedbackState(BeltEvent event);
    void handleErrorState(BeltEvent event);
    
    // Analysis methods
    bool isGoodBraceDetected() const;
    bool isPoorBraceDetected() const;
    float calculateVariance(float* values, int count) const;


    // Calibration methods
    void startCalibration();
    bool isCalibrationComplete();
    void collectCalibrationSample();
    void finishCalibration();


    void startFeedback(FeedbackPattern pattern);
    bool isFeedbackComplete();
    
    // Member variables
    BeltState currentState_;
    unsigned long stateEntryTime_;
    
    // Components (injected dependencies)
    std::unique_ptr<SimpleSensorManager> sensorManager_;
    std::unique_ptr<InputManager> inputManager_;
    std::unique_ptr<FeedbackManager> feedbackManager_;
    
    // Callbacks
    StateChangeCallback stateChangeCallback_;

    unsigned long buttonPressTime_;
    bool waitingForSecondPress_;
    unsigned long calibrationStartTime_;
    unsigned long feedbackStartTime_;
    FeedbackPattern currentFeedbackPattern_;
    
    // Baseline/calibration data
    struct SensorBaseline {
        float fsrBaselines[5];
        float imuBaselines[3][3]; // roll, pitch, yaw for each IMU
        bool isCalibrated;
    } baseline_;

    // Timing
    unsigned long lastEventCheck_;

    int calibrationSampleCount_;
    bool isCollectingCalibration_;
    int calibrationSums_[5];

};