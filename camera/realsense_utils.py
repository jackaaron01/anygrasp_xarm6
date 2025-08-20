# camera/realsense_utils.py
import pyrealsense2 as rs
import numpy as np

def get_realsense_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    profile = pipeline.start(config)
    
    # 添加 RealSense 自带滤波器
    spatial = rs.spatial_filter()   # 空间滤波
    temporal = rs.temporal_filter() # 时间滤波
    hole_filling = rs.hole_filling_filter()  # 孔洞填充
    
    align = rs.align(rs.stream.color)
    return pipeline, align, spatial, temporal, hole_filling

def capture_aligned_frames(pipeline, align, spatial=None, temporal=None, hole_filling=None):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    if not color_frame or not depth_frame:
        return None, None
    # 应用滤波
    if spatial:
        depth_frame = spatial.process(depth_frame)
    if temporal:
        depth_frame = temporal.process(depth_frame)
    if hole_filling:
        depth_frame = hole_filling.process(depth_frame)
    # 转成 numpy 数组
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    return color_image, depth_image
