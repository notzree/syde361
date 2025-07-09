#pragma once
#include "belt_states.h"
#include "belt_events.h"
#include "../sensors/sensor_manager.h"
// #include "../connectivity/connectivity_manager.h"
#include "../feedback/feedback_manager.h"
#include <memory>
#include <functional>
#include <vector>

class BeltFSM {
public:
    using StateChangeCallback = std::function<void(BeltState, BeltState)>;
    
    explicit BeltFSM(
        std::unique_ptr<SensorManager> sensorManager,
        std::unique_ptr<FeedbackManager> feedbackManager
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
    void handlePairingState(BeltEvent event);
    void handleCalibrationState(BeltEvent event);
    void handleCalibratingBaselineState(BeltEvent event);
    void handleCalibratingBraceState(BeltEvent event);
    void handleReadyState(BeltEvent event);
    void handleMonitoringState(BeltEvent event);
    void handleFeedbackGoodState(BeltEvent event);
    void handleFeedbackPoorState(BeltEvent event);
    void handleSleepState(BeltEvent event);
    void handleErrorState(BeltEvent event);
    
    // Analysis methods
    bool isLiftStarted() const;
    bool isLiftComplete() const;
    bool isGoodBraceDetected() const;
    bool isPoorBraceDetected() const;
    bool isActivityDetected() const;
    
    // Calibration methods
    void startBaselineCalibration();
    void startBraceCalibration();
    bool isBaselineComplete() const;
    bool isBraceCalibrationComplete() const;
    
    // Member variables
    BeltState currentState_;
    unsigned long stateEntryTime_;
    unsigned long lastActivity_;
    
    // Components (injected dependencies)
    std::unique_ptr<SensorManager> sensorManager_;
    // std::unique_ptr<ConnectivityManager> connectivityManager_;
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