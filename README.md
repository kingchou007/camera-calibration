# Camera-Calibration

This repository contains tools for calibrating cameras in robotic systems. We support two calibration methods:

1. **Eye-in-Hand Calibration**: For cameras mounted on the robot's end-effector
2. **Eye-to-Hand Calibration**: For cameras mounted externally that observe the robot

## Overview

The calibration process uses ArUco markers as reference points. By capturing multiple views of the marker from different robot poses, the system can calculate the transformation matrix between the camera and the robot.

## Required Topics

For the calibration to work properly, the following ROS topics must be available:

1. **End-Effector Pose Topic** (`/ee_pose` by default)

   - This topic should publish the current pose of the robot's end-effector
   - Message type: `std_msgs/Float64MultiArray`
   - The pose should include position and orientation information
2. **Camera Image Topic** (`/cv_camera/image_raw` by default)

   - This topic should publish the raw images from the camera mounted on the end-effector
   - Message type: `sensor_msgs/Image`
   - The images will be used to detect the ArUco markers
3. **Camera Info Topic** (`/robot_camera_1/intrinsics` by default)

   - This topic should publish the camera's intrinsic parameters
   - Message type: `std_msgs/Float64MultiArray`

You can specify different topic names when running the calibration script using the appropriate command-line options.

## **Prerequisites**

Before running the calibration scripts, make sure the following are installed and properly configured:

1. **ROS (Robot Operating System)**

   Install the appropriate ROS distribution for your system (e.g., ROS Noetic or Melodic). Refer to the [official ROS installation guide](http://wiki.ros.org/ROS/Installation) for details.
2. **Intel RealSense SDK**

   Install the RealSense SDK to connect and manage your RealSense camera. Follow the installation instructions from the [official RealSense GitHub repository](https://github.com/IntelRealSense/librealsense).
3. **ROS RealSense Package**

   Install the ROS wrapper for RealSense cameras:
   bash

   ```bash
   sudo apt-get install ros-<your_ros_distro>-realsense2-camera
   ```

   Replace `<your_ros_distro>` with your ROS version (e.g., `noetic` ).
4. **Other Dependencies**

   Ensure you have the necessary Python packages and tools installed (if any). For example:
   bash

   ```bash
   sudo apt-get install python3-pip
   pip3 install numpy opencv-python
   ```

## **Usage Instructions**

### **Step 1: Launch the RealSense Camera**

Start the RealSense camera using the following command:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This command launches the ROS RealSense node, which streams data from the camera. Make sure the camera is properly connected to your computer.

---

### **Step 2: Run the Calibration Script**

Open a new terminal and execute the calibration script:

```bash
bash run_calibration.sh
```

This script automates the calibration process. Ensure the script is executable by running:

```bash
chmod +x run_calibration.sh
```
