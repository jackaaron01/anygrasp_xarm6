# grasp/grasp_loop.py
import open3d as o3d
import numpy as np
from camera.realsense_utils import capture_aligned_frames
from camera.pointcloud_utils import convert_to_pointcloud
from config.camera_params import fx, fy, cx, cy, workspace_lims

def get_grasp_once(pipeline, align, anygrasp, spatial=None, temporal=None, hole_filling=None, debug=False):
    """
    执行一次抓取检测，返回最优抓取结果或 None
    改进：使用 RealSense 滤波 + 点云降噪 + 下采样
    """
    color, depth = capture_aligned_frames(pipeline, align, spatial, temporal, hole_filling)
    if color is None:
        return None

    # 转换为高质量点云
    points, colors = convert_to_pointcloud(
        color, depth, fx, fy, cx, cy,
        scale=1000.0,
        z_min=0.05,   # 更严格的最小深度，过滤噪点
        z_max=0.8,    # 最大深度
        voxel_size=0.002  # 体素下采样，平衡密度和噪声
    )
    print('[INFO] 点云大小:', points.shape)

    # 抓取预测
    gg, cloud = anygrasp.get_grasp(
        points, colors,
        lims=workspace_lims,
        apply_object_mask=True,
        collision_detection=True
    )

    if gg is None or len(gg) == 0:
        print("[INFO] 未检测到抓取点")
        return None

    # 选取最优抓取
    gg = gg.nms().sort_by_score()
    top_grasp = gg[0:1]
    print(f"[INFO] 最高分抓取点得分: {gg[0].score:.3f}")

    # 坐标系变换（Y、Z 轴取反）
    trans_mat = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    cloud_copy = cloud.transform(trans_mat)
    grippers = top_grasp.to_open3d_geometry_list()
    grippers_transformed = []
    for g in grippers:
        g.transform(trans_mat)
        grippers_transformed.append(g)

    # 调试显示
    if debug:
        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([cloud, *grippers_transformed, axis])

    return top_grasp
