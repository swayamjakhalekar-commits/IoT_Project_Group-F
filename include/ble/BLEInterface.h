#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include "control/Controller.h"
#include "system/SharedState.h"

/**
 * @brief Abstract BLE communication interface
 *
 * Sends structured control commands and updates transmission timestamp.
 */
class BLEInterface {
public:
    virtual ~BLEInterface() = default;

    /**
     * @brief Send control command over BLE
     *
     * @param cmd          Control command (steering + speed)
     * @param shared_state Shared state (for timestamp update)
     */
    virtual void sendCommand(const ControlCommand& cmd,
                             SharedState& shared_state) = 0;
};

#endif
