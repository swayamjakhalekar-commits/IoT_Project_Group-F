#include "control/Controller.h"
#include <algorithm>

ControlCommand Controller::compute(double e_y,
                                   double e_theta,
                                   double fps) {

    ControlCommand cmd;
    cmd.steering = -0.6 * e_y - 0.4 * e_theta;

    double max_speed = fps * 0.02;   // 2cm per frame rule
    cmd.speed = std::clamp(max_speed, 0.0, 0.6);

    return cmd;
}
