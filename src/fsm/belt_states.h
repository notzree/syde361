#pragma once

enum class BeltState {
    STARTUP,
    ERROR,
    IDLE,
    CALIBRATION,
    READY,
    FEEDBACK_GOOD,
    FEEDBACK_BAD
};

const char* stateToString(BeltState state);