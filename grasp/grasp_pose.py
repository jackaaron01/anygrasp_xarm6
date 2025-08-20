# grasp/grasp_pose.py
import numpy as np
from scipy.spatial.transform import Rotation as R

def compute_grasp_pose(top_grasp, arm, tcp_T_camera, debug=True):
    """计算基座系下的抓取位置和姿态"""
    # 获取末端当前姿态
    _, current_pose = arm.get_position(is_radian=False)
    if debug:
        print('末端姿态：', current_pose)

    base_T_tcp = np.eye(4)
    base_T_tcp[:3, 3] = np.array(current_pose[:3]) / 1000.0  # mm -> m
    base_T_tcp[:3, :3] = R.from_euler('xyz', current_pose[3:], degrees=True).as_matrix()
    if debug:
        print('[INFO] 末端位姿矩阵：\n', base_T_tcp)

    # 取最优抓取点
    best_grasp = top_grasp[0]
    if debug:
        print('最佳抓取位姿\n', best_grasp)

    # 相机坐标系下的抓取矩阵
    rot = np.array(best_grasp.rotation_matrix, dtype=float)
    grasp_matrix = np.eye(4)
    grasp_matrix[:3, :3] = rot
    grasp_matrix[:3, 3] = np.array(best_grasp.translation, dtype=float)
    camera_T_grasp = grasp_matrix
    if debug:
        print('[INFO] 相机坐标系下的抓取位姿矩阵:\n', camera_T_grasp)

    # 计算基座坐标系下的抓取位姿
    base_T_grasp = base_T_tcp @ tcp_T_camera @ camera_T_grasp
    if debug:
        print('[INFO] 计算得到基座系抓取矩阵 (m):\n', base_T_grasp)

    grasp_pos_m = base_T_grasp[:3, 3]

    # 姿态修正：绕 Y 轴旋转 90°
    R_y_90 = R.from_euler('y', 90, degrees=True).as_matrix()
    new_rot = base_T_grasp[:3, :3] @ R_y_90
    base_T_grasp[:3, :3] = new_rot

    grasp_rpy_deg = R.from_matrix(new_rot).as_euler('xyz', degrees=True)
    if debug:
        print('[INFO] 计算得到基座系抓取点 (m):', grasp_pos_m)
        print('[INFO] 计算得到基座系欧拉角 (deg):', grasp_rpy_deg)

    return grasp_pos_m, grasp_rpy_deg
