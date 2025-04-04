# Camera-Calibration

Before running the calibration scripts, make sure the following are installed and properly configured:

1. **ROS (Robot Operating System)**

   Install the appropriate ROS distribution for your system (e.g., ROS Noetic). Refer to the [official ROS installation guide](http://wiki.ros.org/ROS/Installation) for details. (Only Support ROS 1 now)
2. **Intel RealSense SDK**

   Install the RealSense SDK to connect and manage your RealSense camera. Follow the installation instructions from the [official RealSense GitHub repository](https://github.com/IntelRealSense/librealsense).
3. **ROS RealSense Package**

   Install the ROS wrapper for RealSense cameras:

```bash
roslaunch realsense2_camera rs_camera.launch
```

open a new terminal:

```bash
bash run_calibration.sh
```
