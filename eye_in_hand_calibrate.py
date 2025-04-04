import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix
import time
from cv_bridge import CvBridge
from flask import Flask, request, jsonify
from geometry_msgs.msg import Pose
import random
import spdlog
from std_msgs.msg import Float64MultiArray
import click
from utils import pose_to_transform

class Targeting:
    def __init__(
        self, marker_id, marker_size, ee_topic, image_topic, camera_info_topic
    ):
        self.logger = spdlog.ConsoleLogger("Targeting")
        self.logger.info("Initializing Calibration node...")

        self.acruco_id = marker_id
        self.marker_size = marker_size
        self.camera_info_loaded = False
        self._cv_bridge = CvBridge()
        self.trans_mats = []
        self.cur_tcp_pose = None
        self.g2r = None
        self.origin_image = None
        self.bgr_image = None
        self.intrinsic_matrix = None
        self.distortion_coefficients = None

        # Initialize subscribers
        self.robot_tcp_position_sub = rospy.Subscriber(
            ee_topic, Float64MultiArray, self._read_tcp_position_sub
        )
        self.rgb_sub = rospy.Subscriber(image_topic, Image, self._bgr_callback)
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            Float64MultiArray,
            self._camera_info_callback
        )
        
        # Initialize publishers
        self.aruco_rgb_pub = rospy.Publisher("/aruco_rgb", Image, queue_size=10)
        self.target_pose_pub = rospy.Publisher("/target_pose", Pose, queue_size=10)

    def _read_tcp_position_sub(self, msg):
        """Callback for robot TCP position"""
        self.cur_tcp_pose = np.array(msg.data)

    def _g2r_callback(self):
        """Calculate gripper to robot base transformation"""
        if self.cur_tcp_pose is None:
            self.logger.error("No TCP pose received yet")
            return
            
        ret = self.cur_tcp_pose
        self.logger.info(f"TCP pose: {ret}")
        transformation_matrix = pose_to_transform(ret, mode="euler")
        self.logger.info(f"Transformation matrix: {transformation_matrix}")
        self.g2r = transformation_matrix

    def _camera_info_callback(self, msg):
        """Process camera intrinsic parameters"""
        if not self.camera_info_loaded:
            self.intrinsic_matrix = {
                'fx': msg.data[0],
                'fy': msg.data[4],
                'cx': msg.data[2],
                'cy': msg.data[5]
            }

            # Optionally get distortion from somewhere if your camera is calibrated
            self.distortion_coefficients = np.zeros(5) 
            self.camera_info_loaded = True
            self.logger.info("Camera intrinsics loaded.")

    def _bgr_callback(self, msg):
        """Process incoming camera images and detect ArUco markers"""
        if not self.camera_info_loaded:
            self.logger.warn("Camera info not loaded yet, skipping frame")
            return
            
        self.origin_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.bgr_image = self.origin_image.copy()

        # Set up ArUco detection
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            self.bgr_image, aruco_dict, parameters=aruco_params
        )
        
        # Create camera matrix from intrinsics
        mtx = np.array([
            [self.intrinsic_matrix["fx"], 0, self.intrinsic_matrix["cx"]],
            [0, self.intrinsic_matrix["fy"], self.intrinsic_matrix["cy"]],
            [0, 0, 1],
        ])

        dist = np.zeros(5)
        
        # Estimate pose of markers
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, mtx, dist
        )

        if ids is not None:
            self.trans_mats = []
            filter_corners = []
            filter_ids = []
            
            # Process each detected marker
            for i, marker_id in enumerate(ids):
                if marker_id == self.acruco_id:
                    rvec, tvec = rvecs[i], tvecs[i]
                    R, _ = cv2.Rodrigues(rvec[0])

                    trans_mat = np.eye(4)
                    trans_mat[:3, :3] = R
                    trans_mat[:3, 3] = tvec
                    cv2.drawFrameAxes(self.bgr_image, mtx, dist, rvec, tvec, 0.05)

                    self.trans_mats.append(trans_mat)
                    filter_corners.append(corners[i])
                    filter_ids.append(ids[i])

            # Draw markers on image
            image_markers = cv2.aruco.drawDetectedMarkers(
                self.bgr_image.copy(), filter_corners, np.array(filter_ids)
            )
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(image_markers, encoding="bgr8")
            )
        else:
            self.logger.error("No AruCo markers detected.")
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(self.bgr_image, encoding="bgr8")
            )

    def vis_targeting(self, test=False):
        """Get the transformation matrix of the detected marker"""
        if not self.trans_mats:
            return None
        return self.trans_mats[0]

    def publish_target_pose(self, transform_matrix):
        """Publish a transformation matrix as a ROS Pose message"""
        target_msg = Pose()
        target_msg.position.x = transform_matrix[0, 3]
        target_msg.position.y = transform_matrix[1, 3]
        target_msg.position.z = transform_matrix[2, 3]
        quaternion = R.from_matrix(transform_matrix[:3, :3]).as_quat()
        target_msg.orientation.x = quaternion[0]
        target_msg.orientation.y = quaternion[1]
        target_msg.orientation.z = quaternion[2]
        target_msg.orientation.w = quaternion[3]
        self.target_pose_pub.publish(target_msg)

    def calibrate(self):
        """Perform hand-eye calibration"""
        o2cs = []  # Object to camera transformations
        g2rs = []  # Gripper to robot base transformations
        
        while True:
            flag = input("Press y to end calibration, or any other key to continue: ")
            if flag == "y":
                break
                
            # Get current transformations
            o2c = self.vis_targeting()
            if o2c is not None:
                o2cs.append(o2c)
                self._g2r_callback()
                if self.g2r is not None:
                    g2r = self.g2r
                    g2rs.append(np.linalg.inv(g2r))
                    self.logger.info(f"Calibration data collected. {len(o2cs)} views.")
                else:
                    self.logger.error("Failed to get gripper pose.")
                    continue
            else:
                self.logger.error("No AruCo markers detected.")
                continue

            # Perform calibration if we have enough data
            if len(o2cs) >= 3:
                R_gripper2base = [g[:3, :3] for g in g2rs]
                t_gripper2base = [g[:3, 3] for g in g2rs]
                R_obj2cam = [o[:3, :3] for o in o2cs]
                t_obj2cam = [o[:3, 3] for o in o2cs]

                # Calculate camera to robot base transformation
                R_cam2base, t_cam2base = cv2.calibrateHandEye(
                    R_gripper2base,
                    t_gripper2base,
                    R_obj2cam,
                    t_obj2cam,
                    method=cv2.CALIB_HAND_EYE_TSAI,
                )

                c2r = np.eye(4)
                c2r[:3, :3] = R_cam2base
                c2r[:3, 3] = t_cam2base[:, 0]

                self.logger.info(f"Current Calibration {len(o2cs)} views. c2r: {c2r}")
                np.save(f"{len(o2cs)}_views_ctor.npy", c2r)
                
                # Publish the transform
                self.publish_target_pose(c2r)
            else:
                # If you have fewer than 3, just give some default transform
                g2c = np.eye(4)
                g2c[2, 3] = 0.3
                self.publish_target_pose(g2c)


def init_ros_node():
    """Initialize ROS node"""
    rospy.init_node('targeting', anonymous=True)


@click.command()
@click.option("--marker_id", default=582, help="Aruco Marker ID (default: 582)")
@click.option(
    "--marker_size", default=0.078, help="Aruco Marker Size in meters (default: 0.078)"
)
@click.option(
    "--ee_topic", default="/ee_pose", help="End-effector pose topic (default: /ee_pose)"
)
@click.option(
    "--image_topic",
    default="/cv_camera/image_raw",
    help="Camera image topic (default: /cv_camera/image_raw)",
)
@click.option(
    "--camera_info_topic",
    default="/robot_camera_1/intrinsics",
    help="Camera info topic (default: /cv_camera/camera_info)",
)
def main(marker_id, marker_size, ee_topic, image_topic, camera_info_topic):
    init_ros_node()
    targeting = Targeting(
        marker_id=marker_id,
        marker_size=marker_size,
        ee_topic=ee_topic,
        image_topic=image_topic,
        camera_info_topic=camera_info_topic,
    )
    time.sleep(2)
    targeting.calibrate()


if __name__ == "__main__":
    main()
