import numpy as np
from scipy.spatial.transform import Rotation as R

def move_along_tcp(x, y, z, rx, ry, rz, distance, axis='z', arm=None, wait=True):
    rot_mat = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    if axis not in axis_map:
        raise ValueError("axis 必须是 'x'、'y' 或 'z'")
    dir_vec = rot_mat[:, axis_map[axis]]
    delta = dir_vec * distance
    target_pos = np.array([x, y, z]) + delta
    target_rpy = [rx, ry, rz]

    if arm is not None:
        arm.set_position(x=target_pos[0]*1000,
                         y=target_pos[1]*1000,
                         z=target_pos[2]*1000,
                         roll=rx, pitch=ry, yaw=rz,
                         wait=wait)
    return target_pos, target_rpy
