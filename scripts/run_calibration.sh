#!/bin/bash

set -e

# Print banner in blue
echo -e "\e[34m============================================"
echo "   Robot-Camera Calibration Script v1.0"
echo "   Authors: Jinzhou Li, Jiyao Zhang"
echo "   Date: $(date +%Y-%m-%d)"
echo -e "============================================\e[0m"

# Launch ROS camera nodes in a new terminal
echo -e "\e[36mLaunching ROS camera nodes...\e[0m"
gnome-terminal -- bash -c "
echo 'Starting ROS camera nodes...';
source /opt/ros/noetic/setup.bash
roslaunch realsense2_camera rs_camera.launch;
exec bash"

# Wait for ROS nodes to initialize
echo -e "\e[36mWaiting for ROS nodes to initialize...\e[0m"
sleep 2

# open a new terminal to run the rqt_image_view
echo -e "\e[36mLaunching rqt_image_view...\e[0m"
gnome-terminal -- bash -c "
echo 'Starting rqt_image_view...';
source /opt/ros/noetic/setup.bash
rqt_image_view;
exec bash"

echo -e "\e[36mWaiting for rqt_image_view to initialize...\e[0m"
sleep 2

# Configuration parameters
MODE=${1:-"eye_to_hand"}  # Default to eye_in_hand if not specified
TEST=${2:-false}          # Default to no test mode
TEST_FILE=${3:-""}        # Optional test file
CAMERA_ID=${4:-"left"}    # Default camera ID to "left" if not specified

# Check if data directory exists, create if not
# Create data and camera-specific directories if they don't exist
DATA_DIR="data"
CAMERA_DATA_DIR="$DATA_DIR/$CAMERA_ID"
if [ ! -d "$CAMERA_DATA_DIR" ]; then
    echo -e "\e[33mCreating directories: $CAMERA_DATA_DIR\e[0m"
    mkdir -p "$CAMERA_DATA_DIR"
fi

# System parameters
MARKER_ID=582
MARKER_SIZE=0.0765
EE_TOPIC="/end_effector_pose"
IMAGE_TOPIC="/camera/color/image_raw"
CAMERA_INFO_TOPIC="/camera/color/camera_info" 
ROTATION_TYPE="quat"

# Validate inputs
if [[ ! "$MODE" =~ ^(eye_in_hand|eye_to_hand)$ ]]; then
    echo -e "\e[31mError: MODE must be either 'eye_in_hand' or 'eye_to_hand'\e[0m"
    exit 1
fi

if [ "$TEST" = true ] && [ -z "$TEST_FILE" ]; then
    echo -e "\e[31mError: Test file must be specified when running in test mode\e[0m"
    exit 1
fi

# Print configuration
echo -e "\n\e[36mConfiguration:"
echo "  Mode: $MODE"
echo "  Test Mode: $TEST"
echo "  Camera ID: $CAMERA_ID"
[ -n "$TEST_FILE" ] && echo "  Test File: $TEST_FILE"
echo -e "\nSystem Parameters:"
echo "  Marker ID: $MARKER_ID"
echo "  Marker Size: $MARKER_SIZE m"
echo "  End-Effector Topic: $EE_TOPIC"
echo "  Image Topic: $IMAGE_TOPIC"
echo "  Camera Info Topic: $CAMERA_INFO_TOPIC"
echo -e "  Rotation Type: $ROTATION_TYPE\e[0m"

echo -e "\n\e[33mStarting calibration...\e[0m\n"

# Build command
CMD="python3 calibration/${MODE}_calibrate.py \
    --marker_id $MARKER_ID \
    --marker_size $MARKER_SIZE \
    --ee_topic $EE_TOPIC \
    --image_topic $IMAGE_TOPIC \
    --camera_info_topic $CAMERA_INFO_TOPIC \
    --rotation_type $ROTATION_TYPE \
    --save_dir $CAMERA_DATA_DIR"

# Add test parameters if needed
if [ "$TEST" = true ]; then
    CMD="$CMD --test --test_file $TEST_FILE"
fi

# Execute calibration
eval $CMD

echo -e "\n\e[36mCalibration completed successfully!\e[0m"
