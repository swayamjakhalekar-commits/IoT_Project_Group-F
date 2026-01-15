#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include "system/SharedState.h"
#include "control/Controller.h"

/**
 * @brief Safety supervisor enforcing real-time and motion constraints
 *
 * - Limits speed based on end-to-end latency
 * - Reduces speed in turns, increases on straights
 * - Stops the car if perception data is stale
 */
class SafetySupervisor {
public:
    virtual ~SafetySupervisor() = default;

    /**
     * @brief Apply safety constraints to control command
     *
     * @param cmd          Control command (speed may be modified)
     * @param shared_state Shared system state (timing + perception)
     */
    void enforceSafety(ControlCommand& cmd,
                       SharedState& shared_state);
};

#endif
