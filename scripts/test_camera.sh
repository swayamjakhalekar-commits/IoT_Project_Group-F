#!/bin/bash

# Full Camera Module Test Script
# Run this on the Linux IoT device

echo "======================================"
echo "  ELGATO CAMERA MODULE - BUILD & TEST"
echo "======================================"
echo ""

# Check if g++ is available
if ! command -v g++ &> /dev/null; then
    echo "✗ g++ not found. Install with: sudo apt-get install build-essential"
    exit 1
fi

# Check if OpenCV is installed
if ! pkg-config --exists opencv4; then
    echo "✗ OpenCV not found. Install with: sudo apt-get install libopencv-dev"
    exit 1
fi

cd /home/iot || cd ~

echo "[STEP 1] Checking camera connection..."
if ! lsusb | grep -i elgato &> /dev/null; then
    echo "⚠ Warning: Elgato not found in lsusb, but continuing..."
fi

if [ ! -e /dev/video0 ]; then
    echo "✗ /dev/video0 not found!"
    exit 1
fi
echo "✓ Camera device found: /dev/video0"
echo ""

echo "[STEP 2] Compiling camera test..."
# Compile with verbose error output
g++ tests/test_camera_full.cpp src/camera/OpenCVCamera.cpp \
    -o test_camera_full \
    -I. -I./include \
    `pkg-config --cflags --libs opencv4` \
    -std=c++17 -Wall -Wextra

if [ $? -ne 0 ]; then
    echo "✗ Compilation failed"
    exit 1
fi
echo "✓ Compilation successful"
echo ""

echo "[STEP 3] Running camera test..."
echo "======================================"
./test_camera_full
TEST_RESULT=$?
echo "======================================"
echo ""

if [ $TEST_RESULT -eq 0 ]; then
    echo "✓ Test completed successfully!"
    echo ""
    echo "✓ Test outputs saved:"
    ls -lh camera_test_frame.jpg 2>/dev/null && echo "  - camera_test_frame.jpg"
    echo ""
    echo "Next: Copy frame to Windows"
    echo "  scp iot@192.168.88.254:/home/iot/camera_test_frame.jpg ."
else
    echo "✗ Test failed with code $TEST_RESULT"
    exit 1
fi
