IPIE Project Structure
IoT_Project_Group-F/
├── CMakeLists.txt
├── README.md
├── config/
│   └── system_config.yaml
├── include/
│   ├── camera/
│   │   ├── CameraInterface.h
│   │   ├── DummyCamera.h
│   │   └── PiCamera.h          # placeholder (future)
│   ├── control/
│   │   ├── Controller.h
│   │   └── PID.h
│   ├── perception/
│   │   └── PerceptionTypes.h
│   ├── ble/
│   │   ├── BLEInterface.h
│   │   └── DummyBLE.h
│   ├── safety/
│   │   └── SafetySupervisor.h
│   └── common/
│       └── TimeUtils.h
├── src/
│   ├── main.cpp
│   ├── camera/
│   │   ├── DummyCamera.cpp
│   │   └── PiCamera.cpp        # placeholder
│   ├── control/
│   │   ├── Controller.cpp
│   │   └── PID.cpp
│   ├── ble/
│   │   └── DummyBLE.cpp
│   ├── safety/
│   │   └── SafetySupervisor.cpp
│   └── common/
│       └── TimeUtils.cpp
└── scripts/
    └── build.sh
