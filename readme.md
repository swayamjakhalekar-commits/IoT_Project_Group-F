# IPIE Autonomous Car Project

This project implements an IoT-based autonomous car system using C++ and CMake. It includes modules for camera capture, perception, control, BLE communication, and safety supervision.

## Project Structure

ipie-autonomous-car/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── config/
│   └── system.yaml
├── include/
│   ├── camera/
│   │   ├── CameraInterface.h
│   │   └── OpenCVCamera.h
│   ├── perception/
│   │   ├── PerceptionTypes.h
│   │   └── TrackDetector.h
│   ├── control/
│   │   └── Controller.h
│   ├── ble/
│   │   └── BLEInterface.h
│   ├── safety/
│   │   └── SafetySupervisor.h
│   └── system/
│       └── SharedState.h
├── src/
│   ├── main.cpp
│   ├── camera/
│   │   └── OpenCVCamera.cpp
│   ├── perception/
│   │   └── TrackDetector.cpp
│   ├── control/
│   │   └── Controller.cpp
│   ├── ble/
│   │   └── DummyBLE.cpp
│   └── safety/
│       └── SafetySupervisor.cpp
└── scripts/
    └── build.sh

























































































































































































































