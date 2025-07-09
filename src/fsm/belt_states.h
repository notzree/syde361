#pragma once

enum class BeltState {
    STARTUP,
    // pairing is for sensors
    PAIRING,
    // start calibration
    // calibrate baseline -> back to calibrating
    // calibrating -> calibrate brace
    CALIBRATING, 
    // Ready state to give feedback
    //
    READY,
    FEEDBACK_GOOD,
    FEEDBACK_POOR,
    ERROR
};

const char* stateToString(BeltState state);