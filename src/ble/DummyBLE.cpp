#include <iostream>
#include <mutex>

#include "control/Controller.h"
#include "system/SharedState.h"
#include "system/TimeUtils.h"

/**
 * @brief Dummy BLE sender (prints command instead of real BLE)
 *
 * @param cmd          Control command
 * @param shared_state Shared system state for timing
 */
void sendBLE(const ControlCommand& cmd, SharedState& shared_state)
{
    // Simulate BLE send
    std::cout << "[BLE] steer=" << cmd.steering
              << " speed=" << cmd.speed << std::endl;

    // Timestamp BLE transmission
    {
        std::lock_guard<std::mutex> lock(shared_state.mtx);
        shared_state.t_ble_ns = TimeUtils::nowNs();
    }
}
