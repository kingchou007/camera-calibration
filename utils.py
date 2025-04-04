from scipy.spatial.transform import Rotation as R
import numpy as np


def pose_to_transform(pose, mode="euler"):
    # TODO: Add type hints
    if mode not in ["euler", "quat"]:
        raise ValueError(f"Invalid mode '{mode}'. Use 'euler' or 'quat'.")

    if mode == "euler" and len(pose) != 6:
        raise ValueError(
            "Pose must have 6 elements [x, y, z, roll, pitch, yaw] for 'euler' mode."
        )
    if mode == "quat" and len(pose) != 7:  # w in the first position or last position
        raise ValueError(
            "Pose must have 7 elements [x, y, z, qx, qy, qz, qw] for 'quat' mode."
        )

    # Compute rotation matrix
    if mode == "euler":
        # Extract Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = pose[3], pose[4], pose[5]
        rot_matrix = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_matrix()
    elif mode == "quat":
        # Extract quaternion (qx, qy, qz, qw)
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        rot_matrix = R.from_quat([qx, qy, qz, qw]).as_matrix()

    # Assign rotation matrix to the transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rot_matrix
    transformation_matrix[:3, 3] = np.array(pose[0:3])

    return transformation_matrix
