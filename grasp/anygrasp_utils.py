import numpy as np
from scipy.spatial.transform import Rotation as R

def build_camera_T_from_grasp(grasp):
    if isinstance(grasp, (list, tuple)) and len(grasp) > 0:
        grasp = grasp[0]

    t = None
    for n in ('translation', 'center', 'position'):
        if hasattr(grasp, n):
            t = np.asarray(getattr(grasp, n)).reshape(3,)
            break

    Rm = None
    if hasattr(grasp, 'rotation_matrix'):
        Rm = np.asarray(grasp.rotation_matrix).reshape(3,3)

    if (t is not None) and (Rm is not None):
        M = np.eye(4)
        M[:3, :3] = Rm
        M[:3, 3] = t
        return M

    raise ValueError("无法解析 grasp 位姿")
