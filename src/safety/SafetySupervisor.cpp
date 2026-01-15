#include "safety/SafetySupervisor.h"

#include <algorithm>
#include <cmath>
#include <mutex>

void SafetySupervisor::enforceSafety(ControlCommand& cmd,
                                     SharedState& shared_state)
{
    uint64_t t_capture = 0;
    uint64_t t_ble = 0;
    double heading_error = 0.0;

    /* =========================
       1. Read shared state
       ========================= */
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        t_capture = shared_state.t_capture_ns;
        t_ble = shared_state.t_ble_ns;
        heading_error = shared_state.heading_error;
    }

    /* =========================
       2. Latency-based safety
       ========================= */

    // If timing not valid yet → stop
    if (t_capture == 0 || t_ble == 0) {
        cmd.speed = 0.0;
        return;
    }

    // End-to-end latency in seconds
    double latency_s =
        static_cast<double>(t_ble - t_capture) * 1e-9;

    constexpr double MAX_LATENCY_S = 0.300; // 300 ms

    // Hard stop if latency too high
    if (latency_s > MAX_LATENCY_S) {
        cmd.speed = 0.0;
        return;
    }

    // Speed limit based on latency
    double latency_safe_speed =
        0.6 * (1.0 - latency_s / MAX_LATENCY_S);

    latency_safe_speed =
        std::clamp(latency_safe_speed, 0.1, 0.6);

    /* =========================
       3. Curvature-based safety
       ========================= */

    // Heading error used as curvature proxy
    double curvature = std::abs(heading_error);

    constexpr double STRAIGHT_TH = 0.05; // almost straight
    constexpr double TURN_TH     = 0.25; // sharp turn

    double curvature_safe_speed = 0.6;

    if (curvature < STRAIGHT_TH) {
        // Straight line → high speed
        curvature_safe_speed = 0.6;
    }
    else if (curvature > TURN_TH) {
        // Sharp turn → slow down
        curvature_safe_speed = 0.2;
    }
    else {
        // Smooth interpolation between straight and turn
        double alpha =
            (curvature - STRAIGHT_TH) / (TURN_TH - STRAIGHT_TH);

        curvature_safe_speed =
            0.6 * (1.0 - alpha) + 0.2 * alpha;
    }

    /* =========================
       4. Final speed decision
       ========================= */

    cmd.speed = std::min({
        cmd.speed,
        latency_safe_speed,
        curvature_safe_speed
    });
}
