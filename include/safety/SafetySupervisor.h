#pragma once

#include "system/SharedState.h"

class SafetySupervisor
{
public:
    /**
     * @brief Apply safety constraints to control command
     */
    void enforceSafety(SharedState::ControlCommand& cmd,
                       SharedState& shared_state);
};
