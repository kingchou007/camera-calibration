#!/bin/bash

set -e
echo "Starting Eye-in-Hand Calibration Script..."

# Modify the following parameters
MODE="eye_to_hand" # eye_in_hand or eye_to_hand
CAMERA_ID="left"    # Default camera ID to "left" if not specified
TEST=True # True or False
PATH_TO_DATA="data/$CAMERA_ID"
TEST_FILE="$PATH_TO_DATA/10_views_ctog.npy"

# define parameters
MARKER_ID=582
MARKER_SIZE=0.0765
EE_TOPIC="/robotic_arm/end_effector_pose"
IMAGE_TOPIC="/robot_camera_1/color_image"
CAMERA_INFO_TOPIC="/robot_camera_1/intrinsics"
ROTATION_TYPE="quat" # euler, quat

# print parameters
echo "  Using the following parameters:"
echo "  Marker ID: $MARKER_ID"
echo "  Marker Size: $MARKER_SIZE m"
echo "  End-Effector Topic: $EE_TOPIC"
echo "  Image Topic: $IMAGE_TOPIC"
echo "  Camera Info Topic: $CAMERA_INFO_TOPIC"
echo "  Rotation Type: $ROTATION_TYPE"
echo "  Test Mode: $TEST"
echo "  Test File: $TEST_FILE"
echo "  Mode: $MODE"

# run the targeting script
python3 calibration/${MODE}_calibrate.py \
    --marker_id $MARKER_ID \
    --marker_size $MARKER_SIZE \
    --ee_topic "$EE_TOPIC" \
    --image_topic "$IMAGE_TOPIC" \
    --camera_info_topic "$CAMERA_INFO_TOPIC" \
    --rotation_type $ROTATION_TYPE \
    --test "$TEST" \
    --test_file "$TEST_FILE"
    
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"