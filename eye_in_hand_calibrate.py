import cv2
import click
import time
import rospy
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
from utils import pose_to_transform
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray


class HandInEyeCalibrator:
    def __init__(
        self, marker_id, marker_size, ee_topic, image_topic, camera_info_topic
    ):
        rospy.loginfo("Initializing Calibration node...")

        self.acruco_id = marker_id
        self.marker_size = marker_size
        self.camera_info_loaded = False
        self._cv_bridge = CvBridge()
        self.trans_mats = []
        self.cur_ee_pose = None
        self.ee2base = None
        self.origin_image = None
        self.bgr_image = None
        self.intrinsic_matrix = None
        self.distortion_coefficients = None

        # Initialize subscribers
        self.ee_position_sub = rospy.Subscriber(
            ee_topic, Float64MultiArray, self._read_ee_position_sub
        )
        self.rgb_sub = rospy.Subscriber(image_topic, Image, self._bgr_callback)
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic, Float64MultiArray, self._camera_info_callback
        )

        # Initialize publishers
        self.aruco_rgb_pub = rospy.Publisher("/aruco_rgb", Image, queue_size=10)
        self.target_pose_pub = rospy.Publisher("/target_pose", Pose, queue_size=10)


    def _read_ee_position_sub(self, msg):
        """Callback for robot end effector position"""
        self.cur_ee_pose = np.array(msg.data)


    def _ee2base_callback(self):
        """Calculate end effector to base transformation"""
        if self.cur_ee_pose is None:
            rospy.logerr("No end effector pose received yet")
            return False

        ret = self.cur_ee_pose
        rospy.loginfo(f"End effector pose: {ret}")
        transformation_matrix = pose_to_transform(ret, mode="euler")
        rospy.loginfo(f"Transformation matrix: {transformation_matrix}")
        self.ee2base = transformation_matrix
        return True


    def _camera_info_callback(self, msg):
        """Process camera intrinsic parameters"""
        if not self.camera_info_loaded:
            self.intrinsic_matrix = {
                "fx": msg.data[0],
                "fy": msg.data[4],
                "cx": msg.data[2],
                "cy": msg.data[5],
            }

            # Optionally get distortion from somewhere if your camera is calibrated
            self.distortion_coefficients = np.zeros(5)
            self.camera_info_loaded = True
            rospy.loginfo("Camera intrinsics loaded.")


    def _bgr_callback(self, msg):
        """Process incoming camera images and detect ArUco markers"""
        if not self.camera_info_loaded:
            rospy.logwarn("Camera info not loaded yet, skipping frame")
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
        mtx = np.array(
            [
                [self.intrinsic_matrix["fx"], 0, self.intrinsic_matrix["cx"]],
                [0, self.intrinsic_matrix["fy"], self.intrinsic_matrix["cy"]],
                [0, 0, 1],
            ]
        )

        dist = self.distortion_coefficients

        # Process detected markers
        if ids is not None:
            # Estimate pose of markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, mtx, dist
            )

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
                    trans_mat[:3, 3] = tvec[0]
                    cv2.drawFrameAxes(self.bgr_image, mtx, dist, rvec, tvec, 0.05)

                    self.trans_mats.append(trans_mat)
                    filter_corners.append(corners[i])
                    filter_ids.append(ids[i])

            # Draw markers on image
            if filter_corners:
                image_markers = cv2.aruco.drawDetectedMarkers(
                    self.bgr_image.copy(), filter_corners, np.array(filter_ids)
                )
                self.aruco_rgb_pub.publish(
                    self._cv_bridge.cv2_to_imgmsg(image_markers, encoding="bgr8")
                )
            else:
                rospy.logwarn(f"ArUco marker ID {self.acruco_id} not found.")
                self.aruco_rgb_pub.publish(
                    self._cv_bridge.cv2_to_imgmsg(self.bgr_image, encoding="bgr8")
                )
        else:
            rospy.logwarn("No ArUco markers detected.")
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(self.bgr_image, encoding="bgr8")
            )


    def vis_targeting(self):
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
        target2cam = []  # Target to camera transformations
        ee2base_list = []  # End effector to base transformations

        while True:
            flag = input("Press y to end calibration, or any other key to continue: ")
            if flag.lower() == "y":
                break

            # Get current transformations
            t2c = self.vis_targeting()
            if t2c is None:
                rospy.logerr("No ArUco markers detected.")
                continue

            if not self._ee2base_callback():
                continue

            target2cam.append(t2c)
            ee2base_list.append(np.linalg.inv(self.ee2base))
            rospy.loginfo(f"Calibration data collected. {len(target2cam)} views.")

            # Perform calibration if we have enough data
            if len(target2cam) >= 3:
                self._perform_calibration(target2cam, ee2base_list)
            else:
                rospy.logwarn(
                    f"Insufficient calibration data. Need at least 3 views, current: {len(target2cam)}"
                )
                rospy.loginfo("Please continue collecting calibration points...")


    def _perform_calibration(self, target2cam, ee2base_list):
        """Execute the hand-eye calibration with collected data"""
        # Extract rotation and translation components
        R_ee2base = [g[:3, :3] for g in ee2base_list]
        t_ee2base = [g[:3, 3] for g in ee2base_list]
        R_target2cam = [o[:3, :3] for o in target2cam]
        t_target2cam = [o[:3, 3] for o in target2cam]

        # Calculate camera to end effector transformation (eye-in-hand)
        R_cam2ee, t_cam2ee = cv2.calibrateHandEye(
            R_ee2base,
            t_ee2base,
            R_target2cam,
            t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )

        # Create the transformation matrix
        cam2ee = np.eye(4)
        cam2ee[:3, :3] = R_cam2ee
        cam2ee[:3, 3] = t_cam2ee.flatten()

        # Save and publish results
        view_count = len(target2cam)
        rospy.loginfo(
            f"Current Calibration with {view_count} views. camera-to-end-effector:\n{cam2ee}"
        )

        # Create directory if it doesn't exist
        os.makedirs("calibration_results", exist_ok=True)
        np.save(f"calibration_results/{view_count}_views_cam2ee.npy", cam2ee)

        # Publish the transform
        self.publish_target_pose(cam2ee)


def init_ros_node():
    """Initialize ROS node"""
    rospy.init_node("targeting", anonymous=True)


@click.command()
@click.option("--marker_id", default=582, help="ArUco Marker ID (default: 582)")
@click.option(
    "--marker_size", default=0.078, help="ArUco Marker Size in meters (default: 0.078)"
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
    help="Camera info topic (default: /camera_info)",
)
def main(marker_id, marker_size, ee_topic, image_topic, camera_info_topic):
    init_ros_node()
    calibrator = HandInEyeCalibrator(
        marker_id=marker_id,
        marker_size=marker_size,
        ee_topic=ee_topic,
        image_topic=image_topic,
        camera_info_topic=camera_info_topic,
    )
    time.sleep(2)  # Wait for subscribers to connect
    calibrator.calibrate()


if __name__ == "__main__":
    main()
