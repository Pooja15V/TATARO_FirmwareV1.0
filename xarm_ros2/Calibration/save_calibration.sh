#!/bin/bash
# Auto-save calibration to project folder after easy_handeye2 completes

CALIB_NAME="${1:-tatro_handeye}"
ROS2_CALIB_PATH="/home/pooja/.ros2/easy_handeye2/calibrations/${CALIB_NAME}.calib"
PROJECT_CALIB_PATH="/home/pooja/TATRO/TATRO_Program_files/TATARO_FirmwareV1.0/xarm_ros2/Calibration/${CALIB_NAME}.calib"

echo "========================================="
echo "TATARO Calibration Auto-Save Script"
echo "========================================="
echo ""
echo "Calibration name: $CALIB_NAME"
echo "Source: $ROS2_CALIB_PATH"
echo "Target: $PROJECT_CALIB_PATH"
echo ""

# Check if source exists
if [ ! -f "$ROS2_CALIB_PATH" ]; then
    echo "❌ ERROR: Calibration file not found at $ROS2_CALIB_PATH"
    echo "Make sure calibration completed successfully!"
    exit 1
fi

# Copy to project folder
cp "$ROS2_CALIB_PATH" "$PROJECT_CALIB_PATH"

if [ $? -eq 0 ]; then
    echo "✅ Calibration saved successfully!"
    echo "Location: $PROJECT_CALIB_PATH"
    echo ""
    echo "To use this calibration:"
    echo "  ros2 launch tataro_packages tatro_runtime.launch.py calib_name:=$CALIB_NAME"
else
    echo "❌ ERROR: Failed to copy calibration file"
    exit 1
fi
