import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation as R
from tf.transformations import quaternion_matrix
import time
from cv_bridge import CvBridge
from flask import Flask, request, jsonify
from geometry_msgs.msg import Pose
import random
import spdlog
from std_msgs.msg import Float64MultiArray

rospy.init_node('targeting', anonymous=True)

class Targeting:
    def __init__(self,):
        self.logger = spdlog.ConsoleLogger("Targeting")

        self.robot_tcp_position_sub = rospy.Subscriber(
            "/robotic_arm/end_effector_pose",
            Float64MultiArray,
            self._read_tcp_position_sub
        )
        self.rgb_sub = rospy.Subscriber(
            "/robot_camera_1/color_image",
            Image,
            self._bgr_callback
        )
        self.camera_info_sub = rospy.Subscriber(
            '/robot_camera_1/intrinsics',
            Float64MultiArray,
            self._camera_info_callback
        )
        self.aruco_rgb_pub = rospy.Publisher('/aruco_rgb', Image, queue_size=10)
        self.target_pose_pub = rospy.Publisher('/target_pose', Pose, queue_size=10)

        self.camera_info_loaded = False
        self._cv_bridge = CvBridge()

        self.marker_size = 0.0765
        self.trans_mats = [None]
        self.acruco_id = 582

        self.cur_tcp_pose = None
        self.g2r = None

    def _read_tcp_position_sub(self, msg):
        self.cur_tcp_pose = np.array(msg.data)

    def _g2r_callback(self):
        if self.cur_tcp_pose is None:
            return
        ret = self.cur_tcp_pose
        rot_matrix = R.from_euler('xyz', [ret[3], ret[4], ret[5]], degrees=False).as_matrix()
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rot_matrix
        transformation_matrix[:3, 3] = np.array(ret[0:3])
        self.logger.info(f'Transformation Matrix (g2r): {transformation_matrix}')
        self.g2r = transformation_matrix

    def _camera_info_callback(self, msg):
        if not self.camera_info_loaded:
            self.intrinsic_matrix = {
                'fx': msg.data[0],
                'fy': msg.data[4],
                'cx': msg.data[2],
                'cy': msg.data[5]
            }
            self.distortion_coefficients = np.zeros(5) 
            self.camera_info_loaded = True
            rospy.loginfo("Camera intrinsics loaded.")

    def _bgr_callback(self, msg):
        if not self.camera_info_loaded:
            return

        self.origin_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.bgr_image = self.origin_image.copy()

        self.bgr_image = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2RGB)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(
            self.bgr_image, aruco_dict, parameters=aruco_params
        )

        fx = self.intrinsic_matrix['fx']
        fy = self.intrinsic_matrix['fy']
        cx = self.intrinsic_matrix['cx']
        cy = self.intrinsic_matrix['cy']
        mtx = np.array([[fx,  0, cx],
                        [ 0, fy, cy],
                        [ 0,  0,  1]], dtype=np.float32)
        dist = np.zeros(5, dtype=np.float32)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                mtx,
                dist
            )

            self.trans_mats = []
            filter_corners = []
            filter_ids = []

            for i, marker_id in enumerate(ids):
                if marker_id == self.acruco_id:
                    rvec = rvecs[i]
                    tvec = tvecs[i]

                    R_mat, _ = cv2.Rodrigues(rvec[0])
                    
                    trans_mat = np.eye(4)
                    trans_mat[:3, :3] = R_mat
                    trans_mat[:3, 3]  = tvec[0]
                    self.trans_mats.append(trans_mat)

                    cv2.drawFrameAxes(
                        self.bgr_image,
                        mtx,
                        dist,
                        rvec,
                        tvec,
                        0.05
                    )
                    filter_corners.append(corners[i])
                    filter_ids.append(ids[i])

            image_markers = cv2.aruco.drawDetectedMarkers(
                self.bgr_image.copy(),
                filter_corners,
                np.array(filter_ids)
            )
            image_markers_bgr = cv2.cvtColor(image_markers, cv2.COLOR_RGB2BGR)
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(image_markers_bgr, encoding='bgr8')
            )

        else:
            self.logger.error("No AruCo markers detected.")
            image_bgr = cv2.cvtColor(self.bgr_image, cv2.COLOR_RGB2BGR)
            self.aruco_rgb_pub.publish(
                self._cv_bridge.cv2_to_imgmsg(image_bgr, encoding='bgr8')
            )

    def vis_targeting(self):
        test = 1
        if test:
            T_camera_to_base = np.load('30views_c2r.npy')
            T_base_to_camera = np.linalg.inv(T_camera_to_base)
            axis_length = 0.1
            axes_points_base = np.array([
                [0, 0, 0],
                [axis_length, 0, 0],
                [0, axis_length, 0],
                [0, 0, axis_length]
            ])

            ones = np.ones((axes_points_base.shape[0], 1))
            axes_points_base_homogeneous = np.hstack([axes_points_base, ones])
            axes_points_camera = (T_base_to_camera @ axes_points_base_homogeneous.T).T
            points_3D = axes_points_camera[:, :3]

            fx = self.intrinsic_matrix['fx']
            fy = self.intrinsic_matrix['fy']
            cx = self.intrinsic_matrix['cx']
            cy = self.intrinsic_matrix['cy']
            mtx = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float32)

            dist = np.zeros(5, dtype=np.float32)
            rvec = np.zeros((3, 1), dtype=np.float32)
            tvec = np.zeros((3, 1), dtype=np.float32)
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

    def calibrate(self,):
        o2cs = []
        g2rs = []
        while True:
            flag = input('press y to end calibration, else continue:')
            if flag == 'y':
                break
            else:
                o2c = self.vis_targeting()
                if o2c is not None:
                    o2cs.append(o2c)
                    self._g2r_callback()
                    g2r = self.g2r
                    g2rs.append(np.linalg.inv(g2r))
                    self.logger.info(f"Calibration data collected. {len(o2cs)} views.")
                else:
                    self.logger.error("No AruCo markers to collect.")

            if len(o2cs) >= 3:
                R_gripper2base = [g[:3, :3] for g in g2rs]
                t_gripper2base = [g[:3, 3] for g in g2rs]
                R_obj2cam = [o[:3, :3] for o in o2cs]
                t_obj2cam = [o[:3, 3] for o in o2cs]

                R_cam2base, t_cam2base = cv2.calibrateHandEye(
                    R_gripper2base,
                    t_gripper2base,
                    R_obj2cam,
                    t_obj2cam,
                    method=cv2.CALIB_HAND_EYE_TSAI
                )
                c2r = np.eye(4)
                c2r[:3, :3] = R_cam2base
                c2r[:3, 3] = t_cam2base[:, 0]

                self.logger.info(f"Current Calibration with {len(o2cs)} views. c2r:\n{c2r}")
                np.save(f'{len(o2cs)}views_c2r.npy', c2r)

                if self.g2r is not None:
                    g2c = np.linalg.inv(c2r) @ self.g2r
                else:
                    g2c = np.eye(4)

            else:
                g2c = np.eye(4)
                g2c[2, 3] = 0.3

            target_msg = Pose()
            target_msg.position.x = g2c[0, 3]
            target_msg.position.y = g2c[1, 3]
            target_msg.position.z = g2c[2, 3]
            quaternion = R.from_matrix(g2c[:3, :3]).as_quat()
            target_msg.orientation.x = quaternion[0]
            target_msg.orientation.y = quaternion[1]
            target_msg.orientation.z = quaternion[2]
            target_msg.orientation.w = quaternion[3]
            self.target_pose_pub.publish(target_msg)

    def calibrate_from_npy(self, views=5):
        for epoch in range(5):
            input('Press Enter to start a new calibration attempt...')
            sample_ids = random.sample(range(20), views)

            o2cs = np.load('o2cs.npy')
            g2rs = np.load('g2rs.npy')

            R_gripper2base = [g2rs[i, :3, :3] for i in sample_ids]
            t_gripper2base = [g2rs[i, :3, 3] for i in sample_ids]
            R_obj2cam = [o2cs[i, :3, :3] for i in sample_ids]
            t_obj2cam = [o2cs[i, :3, 3] for i in sample_ids]

            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_gripper2base,
                t_gripper2base,
                R_obj2cam,
                t_obj2cam,
                method=cv2.CALIB_HAND_EYE_PARK
            )

            c2r = np.eye(4)
            c2r[:3, :3] = R_cam2base
            c2r[:3, 3] = t_cam2base[:, 0]

            if not os.path.exists(f'{views}'):
                os.mkdir(f'{views}')
            np.save(f'{views}/{epoch}_c2r.npy', c2r)


if __name__ == '__main__':
    targeting = Targeting()
    time.sleep(2)
    targeting.calibrate()
