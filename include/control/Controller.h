#pragma once

#include "system/SharedState.h"

struct ControlCommand {
    double steering;
    double speed;
};

class Controller {
public:
    /**
     * @brief Compute control command from perception state
     *
     * @param shared_state Shared system state (errors + timestamps)
     * @return ControlCommand steering + speed
     */
    ControlCommand compute(SharedState& shared_state);
};
