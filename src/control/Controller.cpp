#include "control/Controller.h"

#include <algorithm>
#include <mutex>

ControlCommand Controller::compute(SharedState& shared_state)
{
    ControlCommand cmd{0.0, 0.0};

    double e_y = 0.0;
    double e_theta = 0.0;
    double fps = 0.0;

    /* =========================
       1. Read perception state
       ========================= */
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);

        if (!shared_state.perception_valid)
            return cmd;  // safe fallback

        e_y = shared_state.lateral_error;
        e_theta = shared_state.heading_error;
        fps = shared_state.fps;
    }

    /* =========================
       2. Control law
       ========================= */
    cmd.steering = -0.6 * e_y - 0.4 * e_theta;

    double max_speed = fps * 0.02;   // 2 cm per frame rule
    cmd.speed = std::clamp(max_speed, 0.0, 0.6);

    return cmd;
}
