#pragma once

#include <mutex>
#include <cstdint>

struct SharedState {
    /* ===== Existing control data ===== */
    double lateral_error = 0.0;
    double heading_error = 0.0;
    double fps = 0.0;
    bool perception_valid = false;

    /* ===== Real-time timestamps (nanoseconds) ===== */
    uint64_t t_capture_ns = 0;
    uint64_t t_perception_ns = 0;
    uint64_t t_control_ns = 0;
    uint64_t t_ble_ns = 0;

    /* ===== Frame tracking ===== */
    uint64_t frame_id = 0;

    std::mutex mtx;
};
