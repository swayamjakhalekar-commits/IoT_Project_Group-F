#pragma once
#include <mutex>

struct SharedState {
    std::mutex mtx;

    // perception
    double lateral_error = 0.0;
    double heading_error = 0.0;
    bool perception_valid = false;
    double fps = 0.0;

    // control outputs
    double steering_cmd = 0.0;
    double speed_cmd = 0.0;
};
