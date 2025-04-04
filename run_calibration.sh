#!/bin/bash

set -e
echo "Starting Eye-in-Hand Calibration Script..."

# define parameters
MARKER_ID=582
MARKER_SIZE=0.078
EE_TOPIC="/ee_pose"
IMAGE_TOPIC="/cv_camera/image_raw"
CAMERA_INFO_TOPIC="/cv_camera/camera_info"

# print parameters
echo "Using the following parameters:"
echo "  Marker ID: $MARKER_ID"
echo "  Marker Size: $MARKER_SIZE m"
echo "  End-Effector Topic: $EE_TOPIC"
echo "  Image Topic: $IMAGE_TOPIC"
echo "  Camera Info Topic: $CAMERA_INFO_TOPIC"

# run the targeting script
python3 path/to/your/targeting.py \
    --marker_id $MARKER_ID \
    --marker_size $MARKER_SIZE \
    --ee_topic "$EE_TOPIC" \
    --image_topic "$IMAGE_TOPIC" \
    --camera_info_topic "$CAMERA_INFO_TOPIC"

echo "Eye-in-Hand Calibration Script Finished!"