\# 🚗 Work Process — Vision-Based Autonomous Racing System



This document describes the complete working pipeline of our autonomous racing system, including perception, tracking, control, communication, and system architecture.



\---



\# 🧠 Project Overview



\*\*Project:\*\* Vision-Based Autonomous Model Car  

\*\*Platform:\*\* Raspberry Pi 4  

\*\*Language:\*\* C++17  

\*\*Build System:\*\* CMake  

\*\*Core Idea:\*\* Centralized intelligence using an overhead camera (no sensors on the car)



The system continuously performs:



Read Frame → Detect Car → Compute Control → Send Command



\---



\# Phase 1: System Architecture



\## Components

\- Overhead USB Camera (800×600, 30 FPS)

\- Raspberry Pi 4 (processing unit)

\- BLE RC Car (actuation only)

\- Bluetooth (BLE using BlueZ)



\## Architecture Flow

Camera → Raspberry Pi → BLE → Car



\- All intelligence is offloaded to the Raspberry Pi

\- Car only executes commands



\---



\# Phase 2: Camera \& Frame Acquisition



\## Technologies

\- OpenCV (VideoCapture)

\- Real-time frame streaming



\## Steps

1\. Capture frames using overhead camera  

2\. Convert to HSV color space  

3\. Continuously stream frames to processing pipeline  



\---



\# Phase 3: Track Detection \& Centerline Extraction



\## Technologies

\- OpenCV  

\- HSV Color Segmentation  

\- Curve analysis  



\## Steps



\### 1. Boundary Detection

\- Detect red and white track edges using HSV  

\- Robust against lighting changes  



\### 2. Centerline Computation

\- Compute midpoint between track boundaries  

\- Generate continuous path  



\### 3. Smoothing

\- Apply smoothing to remove noise  



\### 4. Waypoints

\- Divide centerline into evenly spaced points  



\### 5. Speed Zones

\- Assign speed based on curvature:

&#x20; - Straight → High speed  

&#x20; - Mild curve → Medium speed  

&#x20; - Sharp turn → Low speed  



\---



\# Phase 4: Car Detection



\## Technologies

\- MOG2 Background Subtraction  

\- Morphological Operations  

\- Contour Detection  



\## Steps



\### 1. Background Modeling

\- Learn static background (track only)  



\### 2. Foreground Detection

\- Extract moving object (car)  



\### 3. Noise Removal

\- Morphological filtering  



\### 4. Contour Selection

\- Choose largest valid contour  



\### 5. Position Extraction

\- Compute centroid (x, y)  



\---



\# Phase 5: State Estimation (Kalman Filter)



\## State Model

x = \[x, y, vx, vy, ax, ay]



\## Steps



\### 1. Prediction

\- Estimate next position  



\### 2. Update

\- Combine prediction with detection  



\### 3. Smoothing

\- Reduce noise  



\### 4. Lag Compensation

p̂ = p + vΔt + ½aΔt²  



\- Predict future position to compensate delay  



\---



\# Phase 6: Path Tracking (Pure Pursuit)



\## Algorithm

Pure Pursuit Controller  



\## Steps



\### 1. Find Nearest Waypoint

\- Closest point on centerline  



\### 2. Lookahead Target

\- Select point ahead of vehicle  



\### 3. Compute Angle

\- Angle between car direction and target  



\### 4. Steering Calculation

κ = 2 sin(α) / Ld  

δ = arctan(κ · L)  



\### 5. Steering Mapping

\- Convert to control values:

&#x20; - 40 → Right  

&#x20; - 120 → Straight  

&#x20; - 200 → Left  



\### 6. Smoothing

\- Apply smoothing factor (0.65)  



\---



\# Phase 7: Speed Control



\## Logic

\- Based on track curvature  



\## Behavior

\- Accelerate on straights  

\- Slow down before turns  

\- Maintain stability in curves  



\---



\# Phase 8: BLE Communication



\## Technologies

\- Bluetooth Low Energy (BLE)  

\- BlueZ stack (Linux)  



\## Steps



\### 1. Packet Creation

\- 16-byte command packet  

\- Includes speed, steering, torque  



\### 2. Transmission

\- Send via BLE  



\### 3. Acknowledgement

\- Synchronous communication (\~5 ms delay)  



\### 4. Device Handling

\- Connect to available car automatically  



\### 5. Emergency Stop

\- Triggered if car is lost  



\---



\# Phase 9: Safety \& State Machine



\## States

\- SEARCHING → Waiting for detection  

\- LOCKING → Determining direction  

\- RUNNING → Autonomous mode  

\- CAR LOST → Emergency stop  

\- SAFETY HOLD → Re-centering  



\## Purpose

\- Ensures safe and predictable behavior  



\---



\# Phase 10: Multithreading \& Real-Time Execution



\## Technologies

\- std::thread  

\- PREEMPT\_RT (Real-Time Linux)  



\## Threads



\### Tracking Thread

\- Frame capture  

\- Detection + Kalman filter  



\### Control Thread

\- Steering computation  

\- BLE communication  



\## Benefits

\- Parallel execution  

\- Stable 40 Hz control loop  



\---



\# Phase 11: Control Loop Timing



\## Frequency

\- 40 Hz (25 ms cycle)  



\## Timing Breakdown

\- Vision: \~1 ms  

\- Control: \~1 ms  

\- BLE: \~5 ms  

\- Display: \~7 ms  



\*\*Total: \~14 ms per cycle\*\*  



\---



\# Phase 12: Software Architecture



\## Modules

\- tracking\_module.hpp → Car detection  

\- centerline\_module.hpp → Path generation  

\- pure\_pursuit\_controller.hpp → Control logic  

\- safety\_monitor.hpp → Safety system  

\- ble\_manager.hpp → BLE communication  

\- main.cpp → Integration  



\## Design

\- Modular structure  

\- Separation of concerns  



\---



\# Phase 13: Performance



\## Results

\- Stable autonomous laps  

\- Smooth steering  

\- Adaptive speed control  



\## Robustness

\- Handles lighting variation  

\- Recovers from detection noise  

\- Safe failure handling  



\---



\# ✅ Summary



This project demonstrates a real-time autonomous system using:



\- Vision-based perception  

\- Kalman filtering for tracking  

\- Pure pursuit for control  

\- BLE for actuation  



The system is:

\- Fully software-driven  

\- Modular and scalable  

\- Real-time capable  

\- Safe and reliable  

