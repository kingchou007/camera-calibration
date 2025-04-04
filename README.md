# Camera-Calibration

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
