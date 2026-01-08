#pragma once
#include <mutex>

struct SharedState {
    double lateral_error = 0.0;
    double heading_error = 0.0;
    double fps = 0.0;
    bool perception_valid = false;

    std::mutex mtx;
};
