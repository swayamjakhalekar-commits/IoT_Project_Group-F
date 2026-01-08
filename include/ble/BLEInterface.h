#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

#include <string>

class BLEInterface {
public:
    virtual ~BLEInterface() = default;
    virtual void sendData(const std::string& data);
    virtual std::string receiveData();
};

#endif