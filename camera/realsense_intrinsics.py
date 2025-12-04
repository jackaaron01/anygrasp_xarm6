import pyrealsense2 as rs

# 创建配置
pipeline = rs.pipeline()
config = rs.config()

# 选择分辨率，例如 1280×720
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# 启动相机
pipeline.start(config)

# 取一帧
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()

# 获取内参
intr = color_frame.profile.as_video_stream_profile().intrinsics

fx = intr.fx
fy = intr.fy
cx = intr.ppx
cy = intr.ppy

print("fx =", fx)
print("fy =", fy)
print("cx =", cx)
print("cy =", cy)
