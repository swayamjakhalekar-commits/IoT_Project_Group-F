IPIE Project Structure
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