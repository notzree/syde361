#include "belt_states.h"

const char* stateToString(BeltState state) {
    switch (state) {
        case BeltState::STARTUP:
            return "STARTUP";
        case BeltState::ERROR:
            return "ERROR";
        case BeltState::IDLE:
            return "IDLE";
        case BeltState::CALIBRATION:
            return "CALIBRATION";
        case BeltState::READY:
            return "READY";
        case BeltState::FEEDBACK_GOOD:
            return "FEEDBACK_GOOD";
        case BeltState::FEEDBACK_BAD:
            return "FEEDBACK_BAD";
        default:
            return "UNKNOWN";
    }
} 