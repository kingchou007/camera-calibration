from scipy.spatial.transform import Rotation as R
import numpy as np


def pose_to_transform(pose, mode="euler"):
    # TODO: Add type hints
    if mode not in ["euler", "quat"]:
        raise ValueError(f"Invalid mode '{mode}'. Use 'euler' or 'quat'.")

    # Compute rotation matrix
    if mode == "euler":
        # assert len(pose) ==3, "3 elements for Euler angles."
        # # Extract Euler angles (roll, pitch, yaw)
        # roll, pitch, yaw = pose[3], pose[4], pose[5]
        rot_matrix = R.from_euler("xyz", [pose[3], pose[4], pose[5]], degrees=False).as_matrix()
    elif mode == "quat":
        assert len(pose) == 4, "7 elements for quaternion."
        # Extract quaternion (qx, qy, qz, qw)
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        rot_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()

    # Assign rotation matrix to the transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rot_matrix
    transformation_matrix[:3, 3] = np.array(pose[0:3])

    return transformation_matrix
