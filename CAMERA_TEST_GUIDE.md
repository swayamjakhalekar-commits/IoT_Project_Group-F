# Camera Module Test - Quick Start Guide

## Option 1: Manual Testing (Recommended for Debugging)

### On Linux IoT Device:

```bash
# 1. Connect to device
ssh iot@192.168.88.254

# 2. Check camera is connected
lsusb
ls -lh /dev/video0

# 3. Navigate to project
cd /home/iot  # or your project directory

# 4. Compile test
g++ tests/test_camera_full.cpp src/camera/OpenCVCamera.cpp \
    -o test_camera_full \
    -I. -I./include \
    `pkg-config --cflags --libs opencv4` \
    -std=c++17

# 5. Run test
./test_camera_full

# 6. Check output
ls -lh camera_test_frame.jpg
```

---

## Option 2: Automated Testing (From Windows)

### In PowerShell:

```powershell
cd D:\Masters\IoT_Project_Group-F

# Run automated test (uploads, compiles, runs, downloads)
.\test_camera_remote.ps1
```

---

## What the Test Does

✓ **Initializes** the OpenCVCamera with Elgato settings (1280x720)
✓ **Captures** 10 consecutive frames
✓ **Validates** frame dimensions, format, and channels
✓ **Measures** FPS and capture timing
✓ **Saves** first frame as `camera_test_frame.jpg`
✓ **Reports** statistics and any errors

---

## Expected Output

```
====================================
  ELGATO CAMERA MODULE TEST
====================================

[1/5] Initializing OpenCVCamera...
    ✓ Camera initialized in 1.234s

[2/5] Testing frame capture (10 frames)...
--------------------------------------------------
Frame   Resolution      Time (ms)    Status
--------------------------------------------------
0       1280x720        33.45        ✓
1       1280x720        32.12        ✓
2       1280x720        33.78        ✓
3       1280x720        32.89        ✓
4       1280x720        33.21        ✓
5       1280x720        32.56        ✓
6       1280x720        33.44        ✓
7       1280x720        32.78        ✓
8       1280x720        33.12        ✓
9       1280x720        33.01        ✓
--------------------------------------------------

[3/5] Capture Statistics
    Successful frames: 10/10
    Average capture time: 32.98 ms
    Estimated FPS: 30.3 fps

[4/5] Frame validation
    Width: 1280 pixels ✓
    Height: 720 pixels ✓
    Channels: 3 (BGR) ✓
    Data type: 0 (uint8) ✓

[5/5] Saving test frame...
    ✓ Frame saved: camera_test_frame.jpg

====================================
  ✓ CAMERA TEST PASSED
====================================
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `g++: command not found` | Install: `sudo apt-get install build-essential` |
| `pkg-config: command not found` | Install: `sudo apt-get install pkg-config` |
| `opencv4 not found` | Install: `sudo apt-get install libopencv-dev` |
| `/dev/video0 not found` | Check: `lsusb` and `ls /dev/video*` |
| Permission denied on /dev/video0 | Run: `sudo usermod -a -G video $USER` then reboot |
| Frames not 1280x720 | Check camera capabilities: `v4l2-ctl -d 0 --list-formats-ext` |

---

## Next Steps After Passing Test

1. **Copy test frame to Windows:**
   ```powershell
   scp iot@192.168.88.254:/home/iot/camera_test_frame.jpg .
   ```

2. **Test TrackDetector module:**
   ```bash
   g++ tests/test_track_detector.cpp src/perception/TrackDetector.cpp \
       -o test_detector -I. -I./include \
       `pkg-config --cflags --libs opencv4` -std=c++17
   ./test_detector
   ```

3. **Visualize edge detection pipeline:**
   ```bash
   g++ tests/visualize_edges.cpp -o visualize \
       `pkg-config --cflags --libs opencv4` -std=c++17
   ./visualize
   ```
