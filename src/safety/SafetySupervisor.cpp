#include "safety/SafetySupervisor.h"

void SafetySupervisor::enforceSafety(
    SharedState::ControlCommand& cmd,
    SharedState& shared_state)
{
    // Simple example safety rules

    if (!shared_state.perception_valid)
    {
        cmd.speed = 0.0;
        cmd.steering = 0.0;
        return;
    }

    // Clamp steering
    if (cmd.steering > 1.0)
        cmd.steering = 1.0;
    if (cmd.steering < -1.0)
        cmd.steering = -1.0;

    // Clamp speed
    if (cmd.speed > 1.0)
        cmd.speed = 1.0;
    if (cmd.speed < 0.0)
        cmd.speed = 0.0;
}
