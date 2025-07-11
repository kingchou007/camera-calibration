import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
import time
import click
from utils import pose_to_transform
from termcolor import cprint
from geometry_msgs.msg import PoseStamped
import json

rospy.init_node("targeting", anonymous=True)


# ROS Topic Data Types:
# /end_effector_pose (geometry_msgs/PoseStamped):
#   header:
#     seq: sequence ID
#     stamp: timestamp
#     frame_id: reference frame
#   pose:
#     position: (x,y,z)
#     orientation: (x,y,z,w) quaternion

# /camera/color/camera_info (sensor_msgs/CameraInfo):
#   header: 
#     seq: sequence ID
#     stamp: timestamp
#     frame_id: reference frame
#   height: image height
#   width: image width
#   distortion_model: distortion model name
#   D: distortion coefficients
#   K: 3x3 camera intrinsic matrix
#   R: 3x3 rectification matrix
#   P: 3x4 projection matrix

# /camera/color/image_raw (sensor_msgs/Image):
#   header:
#     seq: sequence ID
#     stamp: timestamp
#     frame_id: reference frame
#   height: image height
#   width: image width
#   encoding: pixel encoding
#   is_bigendian: endian order
#   step: row length in bytes
#   data: actual image data


class Targeting:
    def __init__(
        self,
        marker_id,
        marker_size,
        ee_topic,
        image_topic,
        camera_info_topic,
        rotation_type,
        test,
        test_file,
        save_dir,
    ):

        self.acruco_id = marker_id
        self.marker_size = marker_size # Initialize intrinsic matrix

        self.robot_tcp_position_sub = rospy.Subscriber(
            ee_topic, PoseStamped, self._read_tcp_position_sub
        )
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic, CameraInfo, self._camera_info_callback
        )

        self.rgb_sub = rospy.Subscriber(image_topic, Image, self._bgr_callback)
        

        self.aruco_rgb_pub = rospy.Publisher("/aruco_rgb", Image, queue_size=10)
        self.target_pose_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)
        self.rotation_type = rotation_type
        self.test = test
        self.test_file = test_file
        self.save_dir = save_dir
        self.camera_info_loaded = False
        self._cv_bridge = CvBridge()
        self.trans_mats = []


    def _read_tcp_position_sub(self, msg):
        # Convert Pose message to numpy array
        # ! we only accept the quaternion order of w,x,y,z
        pose_array = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.w,  # ROS uses w,x,y,z quaternion order
            msg.pose.orientation.x,
            msg.pose.orientation.y, 
            msg.pose.orientation.z
        ]
        self.cur_tcp_pose = np.array(pose_array)
        # cprint(f"cur_tcp_pose: {self.cur_tcp_pose}", "green")

    def _g2r_callback(self):
        ret = self.cur_tcp_pose
        transformation_matrix = pose_to_transform(ret, mode=self.rotation_type)
        print("transformation_matrix:", transformation_matrix)
        self.g2r = transformation_matrix

    def _camera_info_callback(self, msg):
        if not self.camera_info_loaded:
            self.intrinsic_matrix = {
                "fx": msg.K[0],
                "fy": msg.K[4],
                "cx": msg.K[2],
                "cy": msg.K[5],
            }

            # Optionally get distortion from somewhere if your camera is calibrated
            self.distortion_coefficients = np.zeros(5)
            self.camera_info_loaded = True

    def _bgr_callback(self, msg):            
        self.origin_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.bgr_image = self.origin_image.copy()

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()

        corners, ids, rejected = cv2.aruco.detectMarkers(
            self.bgr_image, aruco_dict, parameters=aruco_params
        )

        mtx = np.array(
            [
                [self.intrinsic_matrix["fx"], 0, self.intrinsic_matrix["cx"]],
                [0, self.intrinsic_matrix["fy"], self.intrinsic_matrix["cy"]],
                [0, 0, 1],
            ]
        )

        dist = np.zeros(5)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, mtx, dist
        )

        if ids is not None:
            self.trans_mats = []
            filter_corners = []
            filter_ids = []
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

            # Draw detected markers
            image_markers = cv2.aruco.drawDetectedMarkers(
                self.bgr_image.copy(), filter_corners, np.array(filter_ids)
            )
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(image_markers, encoding="bgr8")
            )
        else:
            print("NO Markers detected")
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(self.bgr_image, encoding="bgr8")
            )

    def vis_targeting(self, test=False):
        if test:
            T_camera_to_gripper = np.load(self.test_file)

            # get current ee pose

            self._g2r_callback()
            gripper2base = self.g2r
            cprint("please don't move your robot arm!!!", "red")

            T_base_to_camera = np.linalg.inv(T_camera_to_gripper) @ np.linalg.inv(
                gripper2base
            )

            axis_length = 0.1
            axes_points_base = np.array(
                [
                    [0, 0, 0],
                    [axis_length, 0, 0],
                    [0, axis_length, 0],
                    [0, 0, axis_length],
                ]
            )

            ones = np.ones((axes_points_base.shape[0], 1))
            axes_points_base_homogeneous = np.hstack([axes_points_base, ones])

            axes_points_camera = (T_base_to_camera @ axes_points_base_homogeneous.T).T

            points_3D = axes_points_camera[:, :3]

            fx = self.intrinsic_matrix["fx"]
            fy = self.intrinsic_matrix["fy"]
            cx = self.intrinsic_matrix["cx"]
            cy = self.intrinsic_matrix["cy"]

            mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

            dist = np.zeros(5)

            rvec = np.zeros((3, 1))
            tvec = np.zeros((3, 1))

            projected_points, _ = cv2.projectPoints(points_3D, rvec, tvec, mtx, dist)
            projected_points = projected_points.reshape(-1, 2)

            origin = tuple(projected_points[0].astype(int))
            x_axis = tuple(projected_points[1].astype(int))
            y_axis = tuple(projected_points[2].astype(int))
            z_axis = tuple(projected_points[3].astype(int))

            image = self.origin_image.copy()

            cv2.line(image, origin, x_axis, (0, 0, 255), 2)
            cv2.line(image, origin, y_axis, (0, 255, 0), 2)
            cv2.line(image, origin, z_axis, (255, 0, 0), 2)

            cv2.circle(image, origin, radius=5, color=(0, 0, 0), thickness=-1)

            cv2.imshow("Base Position and Orientation", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        if self.trans_mats == []:
            return None

        return self.trans_mats[0]

    # TODO: save all pose
    def calibrate(self):
        o2cs = []  # object to camera: detected from aruco marker -> T_cam_marker
        g2rs = []  # gripper to robot -> T_base_ee
        ee_poses = []  # Store end-effector poses

        # Create data folder if it doesn't exist

        while True:
            flag = input("Press q to end calibration, or any other key to continue:")
            if flag == "q":
                break
            else:
                o2c = self.vis_targeting(test=self.test)
                print("o2c: ", o2c)
                if o2c is not None:
                    o2cs.append(o2c)
                    self._g2r_callback()
                    g2r = self.g2r
                    g2rs.append(g2r)
                    # Store current end-effector pose
                    ee_poses.append(self.cur_tcp_pose.tolist())
                    rospy.loginfo(f"Calibration data collected. {len(o2cs)} views.")
                else:
                    rospy.logwarn("No AruCo markers detected.")

                if len(o2cs) >= 3:
                    R_gripper2base = [g[:3, :3] for g in g2rs]
                    t_gripper2base = [g[:3, 3] for g in g2rs]
                    R_obj2cam = [o[:3, :3] for o in o2cs]
                    t_obj2cam = [o[:3, 3] for o in o2cs]

                    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                        R_gripper2base,
                        t_gripper2base,
                        R_obj2cam,
                        t_obj2cam,
                        method=cv2.CALIB_HAND_EYE_TSAI, 
                    )

                    c2g = np.eye(4)
                    c2g[:3, :3] = R_cam2gripper
                    c2g[:3, 3] = t_cam2gripper[:, 0]

                    rospy.loginfo(f"Current Calibration {len(o2cs)} views.")
                    cprint(f"c2g:\n{c2g}", "green")
                    # Save calibration results
                    np.save(f"{self.save_dir}/{len(o2cs)}_views_ctog.npy", c2g)

                else:
                    # If you have fewer than 3, just give some default transform
                    g2c = np.eye(4)
                    g2c[2, 3] = 0.3

        # Save only the collected end-effector poses used for calibration
        if ee_poses:
            calibration_data = {
                "method": "eye_in_hand",
                "pose_views": len(ee_poses),  # Number of poses collected
                "ee_poses": ee_poses  # List of end-effector poses used in calibration
            }
            
            with open(f"{self.save_dir}/calibration_results_{time.strftime('%Y%m%d_%H%M%S')}.json", 'w') as f:
                json.dump([calibration_data], f, indent=4)


@click.command()
@click.option("--marker_id", "-m", default=582, help="Aruco Marker ID")
@click.option("--marker_size", "-s", default=0.078, help="Marker Size in meters")
@click.option("--ee_topic", "-e", default="/ee_pose", help="End-effector pose topic")
@click.option("--image_topic", "-i", default="/cv_camera/image_raw", help="Camera image topic")
@click.option("--camera_info_topic", "-c", default="/robot_camera_1/intrinsics", help="Camera info topic")
@click.option("--rotation_type", "-r", default="euler", help="Rotation representation type")
@click.option("--test", "-t", is_flag=True, help="Run in test mode")
@click.option("--test_file", "-f", default="10_views_ctog.npy", help="Test calibration file")
@click.option("--save_dir", "-d", default="data", help="Save directory")
def main(
    marker_id,
    marker_size,
    ee_topic,
    image_topic,
    camera_info_topic,
    rotation_type,
    test,
    test_file,
    save_dir,
):
    targeting = Targeting(
        marker_id=marker_id,
        marker_size=marker_size,
        ee_topic=ee_topic,
        image_topic=image_topic,
        camera_info_topic=camera_info_topic,
        rotation_type=rotation_type,
        test=test,
        test_file=test_file,
        save_dir=save_dir,
    )
    time.sleep(2)
    targeting.calibrate()


if __name__ == "__main__":
    main()
