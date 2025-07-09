#pragma once

enum class BeltState {
    STARTUP,
    PAIRING,
    CALIBRATION,
    CALIBRATING_BASELINE,
    CALIBRATING_BRACE,
    READY,
    MONITORING,
    FEEDBACK_GOOD,
    FEEDBACK_POOR,
    SLEEP,
    ERROR
};

const char* stateToString(BeltState state);