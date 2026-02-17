#include <iostream>
#include "system/SharedState.h"

/*
 * Dummy BLE implementation for testing
 * Just prints values instead of sending real BLE
 */

void sendBLE(const SharedState::ControlCommand& cmd,
             SharedState& shared_state)
{
    std::cout << "[BLE] steer=" << cmd.steering
              << " speed=" << cmd.speed
              << std::endl;

    // Update timestamp
    shared_state.t_ble_ns = 0;  // optional dummy value
}
