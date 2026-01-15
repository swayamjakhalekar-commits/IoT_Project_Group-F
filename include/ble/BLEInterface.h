#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include <string>
#include "system/SharedState.h"

class BLEInterface {
public:
    virtual ~BLEInterface() = default;

    /**
     * @brief Send data over BLE and timestamp transmission
     *
     * @param data         Encoded control command
     * @param shared_state Shared system state for timing
     */
    virtual void sendData(const std::string& data,
                          SharedState& shared_state) = 0;

    virtual std::string receiveData() = 0;
};

#endif
