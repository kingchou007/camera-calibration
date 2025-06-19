from scipy.spatial.transform import Rotation as R
import numpy as np


def pose_to_transform(pose, mode="euler"):
    """Convert pose array to 4x4 homogeneous transformation matrix.
    
    The transformation matrix represents both rotation and translation in 3D space.
    For rotation, supports both Euler angles and quaternion representations.
    
    Parameters
    ----------
    pose : array_like
        Array containing position and orientation components:
        - For euler mode: [x, y, z, roll, pitch, yaw] where angles are in radians
        - For quat mode: [x, y, z, qw, qx, qy, qz] where qw is scalar component
    mode : {'euler', 'quat'}, optional
        Specifies rotation representation format:
        - 'euler': Euler angles in xyz convention (intrinsic rotations)
        - 'quat': Quaternion in scalar-last format [w,x,y,z]
        
    Returns
    -------
    transform : ndarray, shape (4, 4)
        Homogeneous transformation matrix:
        [[R R R tx]
         [R R R ty]
         [R R R tz]
         [0 0 0  1]]
        where R is 3x3 rotation matrix and [tx,ty,tz] is translation vector
        
    Raises
    ------
    ValueError
        If pose array length doesn't match specified mode, or mode is invalid
    """
    pose = np.asarray(pose)
    
    # Extract translation vector [x,y,z]
    translation = pose[:3]
    
    # Handle rotation based on mode
    if mode == "euler":
        if len(pose) != 6:
            raise ValueError("Euler pose must have 6 elements [x,y,z,rx,ry,rz]")
        # Convert Euler angles to rotation matrix using xyz convention
        rot_matrix = R.from_euler("xyz", pose[3:], degrees=False).as_matrix()
        
    elif mode == "quat":
        if len(pose) != 7:
            raise ValueError("Quaternion pose must have 7 elements [x,y,z,qw,qx,qy,qz]")
            
        # Extract quaternion [w,x,y,z] and normalize to unit length
        quat = pose[3:]
        quat = quat / np.linalg.norm(quat)
        
        # Convert normalized quaternion to rotation matrix 
        rot_matrix = R.from_quat(quat).as_matrix()
        
    else:
        raise ValueError("Invalid mode. Must be either 'euler' or 'quat'")

    # Construct homogeneous transformation matrix
    transform = np.empty((4, 4))
    transform[:3, :3] = rot_matrix  # Set rotation block
    transform[:3, 3] = translation  # Set translation vector
    transform[3] = [0, 0, 0, 1]     # Set homogeneous row

    return transform
