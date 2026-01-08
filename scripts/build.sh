#!/bin/bash

echo "Building IPIE Autonomous Car..."

mkdir -p build
cd build
cmake ..
make

echo "Build complete."