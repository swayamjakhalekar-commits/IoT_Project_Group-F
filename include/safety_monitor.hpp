#pragma once
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include "ble_manager.hpp"
#include "tracking_module.hpp"

// ============================================================
// SAFETY MONITOR  v8  —  background thread at ~5 Hz
//
// Trips (stopImmediate + safetyTripped=true) if:
//   1. Car detection lost for > 1.5 s
//   2. BLE connection drops
//
// Recovery: attempts BLE reconnect every poll cycle.
// The control loop checks safetyTripped each frame and
// shows a hold screen instead of sending BLE commands.
// ============================================================

class SafetyMonitor {
public:
    atomic<bool> running{false};
    atomic<bool> safetyTripped{false};

    static constexpr double TIMEOUT_SEC = 1.5;

    void run(BLEManager& ble, TrackingModule& tracker, bool clReady) {
        cout << "[Safety] Started.\n";
        if (!clReady) {
            cerr << "[Safety] Centerline not ready — abort.\n";
            ble.stopImmediate();
            safetyTripped = true;
            return;
        }

        while (running) {
            double now = chrono::duration<double>(
                chrono::steady_clock::now().time_since_epoch()).count();

            double lastSeen = tracker.lastDetectionTime.load();
            if (lastSeen > 0.0 && (now - lastSeen) > TIMEOUT_SEC) {
                if (!safetyTripped) {
                    cerr << "[Safety] Car lost >1.5s — STOP.\n";
                    ble.stopImmediate();
                    safetyTripped = true;
                }
            }

            if (!ble.checkConnection()) {
                if (!safetyTripped) {
                    cerr << "[Safety] BLE dropped — STOP.\n";
                    ble.stopImmediate();
                    safetyTripped = true;
                }
            }

            if (safetyTripped && ble.connect(2)) {
                cout << "[Safety] Reconnected — resuming.\n";
                safetyTripped = false;
            }

            this_thread::sleep_for(chrono::milliseconds(200));
        }
        cout << "[Safety] Stopped.\n";
    }
};
