#include "control/Controller.h"
#include <iostream>

void Controller::control(double speed, double steering) {
    std::cout << "Controlling: speed=" << speed << ", steering=" << steering << std::endl;
}