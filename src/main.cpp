#include <iostream>

#include "system/SharedState.h"
#include "camera/OpenCVCamera.h"
#include "perception/TrackDetector.h"
#include "control/Controller.h"
#include "ble/BLEInterface.h"
#include "safety/SafetySupervisor.h"

int main() {
    std::cout << "IPIE Autonomous Car Starting" << std::endl;

    SharedState state;
    OpenCVCamera camera;
    TrackDetector detector;
    Controller controller;
    BLEInterface ble;
    SafetySupervisor safety;

    // Simple simulation loop
    for(int i = 0; i < 5; ++i) {
        camera.capture();
        Track track = detector.detectTrack();
        controller.control(1.0, 0.0);
        ble.sendData("status update");
        std::string data = ble.receiveData();
        bool safe = safety.checkSafety();
        std::cout << "Iteration " << i << ": Safe = " << (safe ? "Yes" : "No") << std::endl;
    }

    return 0;
}