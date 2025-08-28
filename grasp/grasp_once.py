# grasp/grasp_loop.py
import numpy as np
import open3d as o3d
from grasp.grasp_pose import compute_grasp_pose

def get_grasp_once(pipeline, align, anygrasp,
                   spatial=None, temporal=None, hole_filling=None,
                   debug=False, roi=None, save_ply=False):
    """
    从 RealSense 获取一帧并计算抓取点
    :param roi: (x1, y1, x2, y2)，YOLO 检测到的物体框，用于裁剪点云
    """
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame:
        return None

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # ROI 裁剪
    if roi is not None:
        x1, y1, x2, y2 = roi
        depth_image = depth_image[y1:y2, x1:x2]
        color_image = color_image[y1:y2, x1:x2]
        color_crop = np.ascontiguousarray(color_image[y1:y2, x1:x2])
        if debug:
            print(f"[DEBUG] 使用 ROI 裁剪: {roi}, 剩余尺寸={depth_image.shape}")

    # 转点云
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color_image),
        o3d.geometry.Image(depth_image),
        depth_scale=1000.0,
        depth_trunc=1.0,
        convert_rgb_to_intensity=False
    )
    intrinsics = aligned_frames.profile.as_video_stream_profile().intrinsics
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width, intrinsics.height,
        intrinsics.fx, intrinsics.fy,
        intrinsics.ppx, intrinsics.ppy
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, pinhole_camera_intrinsic
    )
    pcd = pcd.voxel_down_sample(voxel_size=0.002)

    if save_ply:
        o3d.io.write_point_cloud("capture.ply", pcd)

    # 送入 AnyGrasp 推理
    grasps = anygrasp.predict_grasps(pcd)
    if not grasps or len(grasps) == 0:
        return None

    return select_top_grasp(grasps, debug=debug)
