#!/bin/bash

# ======================================
#  ELGATO CAMERA MODULE - BUILD & TEST
# ======================================

set -e  # Stop immediately if any command fails

echo ""
echo "======================================"
echo "  ELGATO CAMERA MODULE - BUILD & TEST"
echo "======================================"
echo ""

# Check if g++ exists
if ! command -v g++ &> /dev/null; then
    echo "✗ g++ not found. Install with:"
    echo "  sudo apt-get install build-essential"
    exit 1
fi

# Check if OpenCV exists
if ! pkg-config --exists opencv4; then
    echo "✗ OpenCV not found. Install with:"
    echo "  sudo apt-get install libopencv-dev"
    exit 1
fi

echo "[STEP 1] Checking camera device..."

if [ ! -e /dev/video0 ]; then
    echo "✗ /dev/video0 not found!"
    exit 1
fi

echo "✓ Camera device found: /dev/video0"
echo ""

echo "[STEP 2] Compiling camera test..."

g++ tests/test_camera_full.cpp src/camera/OpenCVCamera.cpp \
    -o test_camera_full \
    -Iinclude \
    `pkg-config --cflags --libs opencv4` \
    -std=c++17 -Wall -Wextra

echo "✓ Compilation successful"
echo ""

echo "[STEP 3] Running camera test..."
echo "======================================"

./test_camera_full

echo "======================================"
echo ""

echo "✓ Test completed successfully!"

if [ -f camera_test_frame.jpg ]; then
    echo "✓ Output saved:"
    ls -lh camera_test_frame.jpg
fi

echo ""
echo "To copy frame to Windows:"
echo "scp <user>@<pi-ip>:/path/to/camera_test_frame.jpg ."
echo ""
