#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>

// ============================================================
// BLE MANAGER  v8  —  Dual Car Support
//
// CARS:
//   RED CAR:  ED:5C:23:84:48:8D
//   GREY CAR: F9:AF:3C:E2:D2:F5
//
// AUTO-SELECTION:
//   On connect(), tries RED car first, then GREY car.
//   Uses whichever is available.
//
// Frame byte map:
//   [7]  THROTTLE   0 = stop  |  80 = full forward  |  200 = full reverse
//   [11] STEER      40 = full right  |  120 = centre  |  200 = full left
//   [12] TORQUE     32 = active  |  0 = idle
//
// Throttle values used in this project:
//   THR_WIGGLE = 5   — tiny wiggle during car detection
//   THR_ALIGN  = 8   — gentle alignment pulses
//   THR_DRIVE  = 10  — main autonomous loop
//
// stopImmediate():
//   1. Purges all pending frames from the queue
//   2. Sends BASE_FRAME synchronously via direct busctl call
//   This guarantees the car halts NOW — not after draining a
//   queue of stale commands that each take ~300ms to execute.
//
// BLE is capped at ~15 Hz (every 2nd control frame) to prevent
// queue buildup that causes delayed stops on 'q'.
// ============================================================

class BLEManager {
public:
    // ═══════════════════════════════════════════════════════════
    // DUAL CAR CONFIGURATION
    // ═══════════════════════════════════════════════════════════
    struct CarConfig {
        const char* name;
        const char* mac;
        const char* gattPath;
    };

    static constexpr CarConfig RED_CAR = {
        "RED CAR",
        "ED:5C:23:84:48:8D",
        "/org/bluez/hci0/dev_ED_5C_23_84_48_8D/service000b/char000f"
    };

    static constexpr CarConfig GREY_CAR = {
        "GREY CAR",
        "F9:AF:3C:E2:D2:F5",
        "/org/bluez/hci0/dev_F9_AF_3C_E2_D2_F5/service000b/char000f"
    };

    // Currently active car (set during connect)
    std::string activeMac;
    std::string activeGattPath;
    std::string activeCarName;

    // Neutral / idle frame — all bytes from hardware capture
    const std::vector<int> BASE_FRAME = {191,10,0,8,40,0,0,0,0,0,0,0,0,0,0,0};

    static constexpr int THROTTLE_BYTE = 7;
    static constexpr int STEER_BYTE    = 11;
    static constexpr int TORQUE_BYTE   = 12;

    static constexpr int THR_STOP   = 0;
    static constexpr int THR_WIGGLE = 5;
    static constexpr int THR_ALIGN  = 8;
    static constexpr int THR_DRIVE  = 10;

    std::atomic<bool> connected{false};

    BLEManager() {
        // Default to grey car (will be overridden by connect())
        activeMac = GREY_CAR.mac;
        activeGattPath = GREY_CAR.gattPath;
        activeCarName = GREY_CAR.name;
    }

    void startSenderThread() {
        senderRunning = true;
        senderThread  = std::thread(&BLEManager::senderLoop, this);
        std::cout << "[BLE] Sender thread started.\n";
    }

    void stopSenderThread() {
        senderRunning = false;
        cv.notify_all();
        if (senderThread.joinable()) senderThread.join();
        std::cout << "[BLE] Sender thread stopped.\n";
    }

    // Check if a specific MAC is connected
    bool checkConnectionFor(const std::string& mac) {
        std::string cmd = "bluetoothctl info " + mac
                        + " 2>/dev/null | grep -q 'Connected: yes'";
        return (system(cmd.c_str()) == 0);
    }

    // Check current active car connection
    bool checkConnection() {
        connected = checkConnectionFor(activeMac);
        return connected.load();
    }

    // Try to connect to a specific car
    bool tryConnectCar(const CarConfig& car, int maxRetries = 3) {
        std::cout << "[BLE] Trying " << car.name << " (" << car.mac << ")...\n";
        
        for (int i = 0; i < maxRetries; ++i) {
            // First check if already connected
            if (checkConnectionFor(car.mac)) {
                std::cout << "[BLE] " << car.name << " already connected ✓\n";
                activeMac = car.mac;
                activeGattPath = car.gattPath;
                activeCarName = car.name;
                connected = true;
                return true;
            }

            // Try to connect
            std::string cmd = "bluetoothctl connect " + std::string(car.mac) + " > /dev/null 2>&1";
            system(cmd.c_str());
            sleep(2);

            if (checkConnectionFor(car.mac)) {
                std::cout << "[BLE] " << car.name << " connected ✓\n";
                activeMac = car.mac;
                activeGattPath = car.gattPath;
                activeCarName = car.name;
                connected = true;
                return true;
            }
            std::cout << "[BLE] " << car.name << " retry " << i+1 << "/" << maxRetries << "\n";
        }
        return false;
    }

    // ═══════════════════════════════════════════════════════════
    // AUTO-SELECT CONNECTION
    // Tries RED car first, then GREY car
    // ═══════════════════════════════════════════════════════════
    bool connect(int maxRetries = 5) {
        std::cout << "[BLE] Auto-selecting car...\n";
        std::cout << "[BLE] Available cars:\n";
        std::cout << "      1. " << RED_CAR.name << " (" << RED_CAR.mac << ")\n";
        std::cout << "      2. " << GREY_CAR.name << " (" << GREY_CAR.mac << ")\n\n";

        // Try RED car first
        if (tryConnectCar(RED_CAR, maxRetries)) {
            std::cout << "\n[BLE] ═══════════════════════════════════════\n";
            std::cout << "[BLE]   ACTIVE CAR: " << activeCarName << "\n";
            std::cout << "[BLE]   MAC: " << activeMac << "\n";
            std::cout << "[BLE] ═══════════════════════════════════════\n\n";
            return true;
        }

        std::cout << "[BLE] RED CAR not available, trying GREY CAR...\n";

        // Try GREY car
        if (tryConnectCar(GREY_CAR, maxRetries)) {
            std::cout << "\n[BLE] ═══════════════════════════════════════\n";
            std::cout << "[BLE]   ACTIVE CAR: " << activeCarName << "\n";
            std::cout << "[BLE]   MAC: " << activeMac << "\n";
            std::cout << "[BLE] ═══════════════════════════════════════\n\n";
            return true;
        }

        std::cerr << "[BLE] No cars available! Connection failed.\n";
        return false;
    }

    // Force connect to a specific car
    bool connectTo(const CarConfig& car, int maxRetries = 5) {
        std::cout << "[BLE] Force connecting to " << car.name << "...\n";
        if (tryConnectCar(car, maxRetries)) {
            std::cout << "[BLE] Connected to " << activeCarName << " ✓\n";
            return true;
        }
        std::cerr << "[BLE] Failed to connect to " << car.name << "\n";
        return false;
    }

    // Non-blocking enqueue — latest-wins (depth = 1)
    void sendFrame(const std::vector<int>& frame) {
        {
            std::lock_guard<std::mutex> lk(queueMutex);
            while (!commandQueue.empty()) commandQueue.pop();
            commandQueue.push(frame);
        }
        cv.notify_one();
    }

    // ── GUARANTEED IMMEDIATE STOP ──────────────────────────────
    // Purge queue first, then send stop synchronously (blocks ~300ms).
    // Must be called on 'q', car-lost, and shutdown.
    void stopImmediate() {
        {
            std::lock_guard<std::mutex> lk(queueMutex);
            while (!commandQueue.empty()) commandQueue.pop();
        }
        execBusctl(BASE_FRAME);   // direct, not via thread
        std::cout << "[BLE] stopImmediate sent.\n";
    }

    void stop() { stopImmediate(); }

    // Blocking pulse: send frame for pulseMs, then stop + 150ms gap
    void pulse(int throttle, int steer, int pulseMs) {
        auto f = BASE_FRAME;
        f[THROTTLE_BYTE] = throttle;
        f[STEER_BYTE]    = steer;
        f[TORQUE_BYTE]   = 32;
        sendFrame(f);
        std::this_thread::sleep_for(std::chrono::milliseconds(pulseMs));
        stopImmediate();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    // Get active car info for display
    std::string getActiveCarInfo() const {
        return activeCarName + " (" + activeMac + ")";
    }

private:
    std::thread                  senderThread;
    std::atomic<bool>            senderRunning{false};
    std::queue<std::vector<int>> commandQueue;
    std::mutex                   queueMutex;
    std::condition_variable      cv;

    void execBusctl(const std::vector<int>& frame) {
        std::string cmd =
            "sudo busctl call org.bluez " + activeGattPath
            + " org.bluez.GattCharacteristic1 WriteValue aya{sv} "
            + std::to_string(frame.size());
        for (int b : frame) cmd += " " + std::to_string(b);
        cmd += " 1 type s command";
        system(cmd.c_str());
    }

    void senderLoop() {
        while (senderRunning) {
            std::vector<int> frame;
            {
                std::unique_lock<std::mutex> lk(queueMutex);
                cv.wait(lk, [this]{
                    return !commandQueue.empty() || !senderRunning;
                });
                if (!senderRunning) break;
                frame = commandQueue.front();
                commandQueue.pop();
            }
            execBusctl(frame);
        }
    }
};
