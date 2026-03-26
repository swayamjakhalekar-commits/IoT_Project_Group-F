# 🚗 Work Process — Autonomous Racing System

This document explains the complete workflow of our vision-based autonomous racing system, from perception to control and communication.

---

## 🧠 Overview

The system enables a model car to drive autonomously using a **single overhead camera** and a **Raspberry Pi**.

### 🔁 Control Pipeline

Camera Input → Car Detection → State Estimation → Path Tracking → BLE Command


- No sensors on the car  
- All intelligence is centralized  
- Runs in real-time (~40 Hz)

---

## 🏗️ System Architecture

### Components

- 📷 Overhead Camera (800×600 @ 30 FPS)
- 🧠 Raspberry Pi 4 (processing)
- 🚗 BLE RC Car (actuation only)
- 📡 Bluetooth (BLE via BlueZ)

### Data Flow

Camera → Raspberry Pi → BLE → Car

---

## 📸 Perception Module (Tracking)

### Technologies
- OpenCV
- MOG2 Background Subtraction
- Contour Detection

### Workflow

1. Capture frame from camera  
2. Apply background subtraction (MOG2)  
3. Remove noise using morphological operations  
4. Detect contours  
5. Extract car position (centroid)

---

## 🛣️ Track Detection & Centerline

### Technologies
- HSV Color Segmentation
- OpenCV Processing

### Workflow

- Detect **red + white boundaries**
- Compute midpoint → centerline
- Smooth the path
- Generate waypoints

### 🚦 Speed Zones

| Zone | Description | Speed |
|------|------------|------|
| Straight | Low curvature | High |
| Curve | Moderate curvature | Medium |
| Sharp Turn | High curvature | Low |

---

## 📍 State Estimation (Kalman Filter)

### State Vector

[x, y, vx, vy, ax, ay]


### Purpose

- Smooth noisy detections  
- Predict motion  
- Handle temporary detection loss  

### Prediction

p̂ = p + vΔt + ½aΔt²

---

## 🎯 Path Tracking (Pure Pursuit)

### Algorithm
Pure Pursuit Controller

### Steps

1. Find nearest waypoint  
2. Select lookahead target  
3. Compute steering angle  

### Steering Model

κ = 2 sin(α) / Ld
δ = arctan(κ · L)


### Output Mapping

- 40 → Full Right  
- 120 → Straight  
- 200 → Full Left  

### Smoothing
- Steering smoothing factor: **0.65**

---

## ⚡ Speed Control

### Strategy
- Based on track curvature

### Behavior

- Accelerate on straights  
- Slow down before turns  
- Maintain stability in curves  

---

## 📡 BLE Communication

### Technologies
- Bluetooth Low Energy (BLE)
- BlueZ (Linux)

### Workflow

1. Build 16-byte command packet  
2. Include:
   - Speed  
   - Steering  
   - Torque  
3. Send via BLE  
4. Wait for acknowledgment (~5 ms)

### 🛑 Safety
- Emergency stop if car is lost  

---

## 🛡️ Safety System

### State Machine

| State | Description |
|------|------------|
| SEARCHING | Waiting for car |
| LOCKING | Detecting direction |
| RUNNING | Autonomous control |
| CAR LOST | Emergency stop |
| SAFETY HOLD | Re-centering |

---

## ⚙️ Multithreading

### Threads

#### 1. Tracking Thread
- Frame capture  
- Detection  
- Kalman filtering  

#### 2. Control Thread
- Path tracking  
- Speed control  
- BLE communication  

### Benefits
- Parallel execution  
- Stable real-time performance  

---

## ⏱️ Performance

### Control Loop
- Frequency: **40 Hz (25 ms)**

### Timing Breakdown

| Task | Time |
|------|------|
| Vision | ~1 ms |
| Control | ~1 ms |
| BLE | ~5 ms |
| Display | ~7 ms |
| **Total** | ~14 ms |

---

## 🧩 Software Modules

- `tracking_module.hpp` → Car detection  
- `centerline_module.hpp` → Path generation  
- `pure_pursuit_controller.hpp` → Control  
- `safety_monitor.hpp` → Safety logic  
- `ble_manager.hpp` → BLE communication  
- `main.cpp` → Integration  

---

## 📊 Key Features

- ✅ Real-time autonomous control  
- ✅ Vision-based tracking (no sensors)  
- ✅ Smooth steering (Pure Pursuit)  
- ✅ Robust tracking (Kalman filter)  
- ✅ Safe operation (state machine)  
- ✅ Modular architecture  

---

## 🚀 Summary

This system demonstrates a **complete autonomous pipeline** using:

- Computer vision for perception  
- Kalman filtering for tracking  
- Pure pursuit for control  
- BLE for wireless actuation  

The result is a **stable, real-time, and fully autonomous racing system** built using low-cost hardware.

---


