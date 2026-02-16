#!/bin/bash

echo "======================================"
echo " Building IPIE Autonomous Car..."
echo "======================================"

set -e  # Stop script if any command fails

mkdir -p build
cd build

# Configure only if needed
if [ ! -f "Makefile" ]; then
    echo "[1/3] Running CMake configuration..."
    cmake ..
else
    echo "[1/3] CMake already configured."
fi

echo "[2/3] Building project..."
make -j$(nproc)

echo "[3/3] Build complete."
echo "======================================"
