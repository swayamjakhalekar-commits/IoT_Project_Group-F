#pragma once

#include "system/SharedState.h"

class Controller {
public:
    /**
     * @brief Compute control command from perception state
     * @param shared_state Shared system state
     * @return SharedState::ControlCommand
     */
    SharedState::ControlCommand compute(SharedState& shared_state);
};
