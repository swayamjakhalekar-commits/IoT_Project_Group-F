#pragma once

#include "system/SharedState.h"

class Controller {
public:
    /**
     * @brief Compute control command from perception state
     * @param shared_state Shared system state (errors + timestamps)
     * @return SharedState::ControlCommand steering + speed
     */
    SharedState::ControlCommand compute(SharedState& shared_state);
};
