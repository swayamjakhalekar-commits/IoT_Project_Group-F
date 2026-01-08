#include <iostream>
#include "control/Controller.h"

void sendBLE(const ControlCommand& cmd) {
    std::cout << "[BLE] steer=" << cmd.steering
              << " speed=" << cmd.speed << std::endl;
}
