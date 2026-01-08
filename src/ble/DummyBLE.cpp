#include "ble/BLEInterface.h"
#include <iostream>

void BLEInterface::sendData(const std::string& data) {
    std::cout << "BLE sending: " << data << std::endl;
}

std::string BLEInterface::receiveData() {
    return "received BLE data";
}