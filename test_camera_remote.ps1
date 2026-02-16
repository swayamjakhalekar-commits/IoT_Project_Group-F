# PowerShell script to test camera module on IoT device
# Usage: .\test_camera_remote.ps1

$device_ip = "192.168.88.254"
$device_user = "iot"
$project_path = "/home/iot"

Write-Host "======================================"
Write-Host "  CAMERA TEST - Remote Execution"
Write-Host "======================================"
Write-Host ""

# Step 1: Upload test script
Write-Host "[1/4] Uploading test files..."
scp -r .\tests\test_camera_full.cpp "${device_user}@${device_ip}:${project_path}/tests/"
scp -r .\src\camera\OpenCVCamera.cpp "${device_user}@${device_ip}:${project_path}/src/camera/"
scp .\scripts\test_camera.sh "${device_user}@${device_ip}:${project_path}/scripts/"
Write-Host "✓ Files uploaded"
Write-Host ""

# Step 2: Make script executable and run
Write-Host "[2/4] Executing test on device..."
ssh "${device_user}@${device_ip}" "cd ${project_path} && chmod +x scripts/test_camera.sh && bash scripts/test_camera.sh"

Write-Host ""
Write-Host "[3/4] Downloading test frame..."
scp "${device_user}@${device_ip}:${project_path}/camera_test_frame.jpg" .
Write-Host "✓ Frame downloaded: camera_test_frame.jpg"
Write-Host ""

Write-Host "[4/4] Test complete!"
Write-Host "  View frame: camera_test_frame.jpg"
Write-Host ""
