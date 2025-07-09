#include "belt_states.h"

const char* stateToString(BeltState state) {
    switch (state) {
        case BeltState::STARTUP:
            return "STARTUP";
        case BeltState::PAIRING:
            return "PAIRING";
        case BeltState::CALIBRATING:
            return "CALIBRATING";
        case BeltState::READY:
            return "READY";
        case BeltState::FEEDBACK_GOOD:
            return "FEEDBACK_GOOD";
        case BeltState::FEEDBACK_POOR:
            return "FEEDBACK_POOR";
        case BeltState::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
} 