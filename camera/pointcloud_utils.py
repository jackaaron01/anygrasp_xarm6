# camera/pointcloud_utils.py
import numpy as np
import open3d as o3d

def convert_to_pointcloud(color, depth, fx, fy, cx, cy, scale=1000.0, z_min=0.1, z_max=0.8, voxel_size=0.002):
    h, w = depth.shape
    xmap, ymap = np.meshgrid(np.arange(w), np.arange(h))
    z = depth / scale
    x = (xmap - cx) / fx * z
    y = (ymap - cy) / fy * z
    
    mask = (z > z_min) & (z < z_max)
    points = np.stack((x, y, z), axis=-1)[mask]
    colors = (color / 255.0)[mask]
    
    # 转成 Open3D 点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 体素下采样 + 统计滤波
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
    return np.asarray(pcd.points, dtype=np.float32), np.asarray(pcd.colors, dtype=np.float32)

def save_pointcloud_to_ply(points, colors, filename):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(filename, pcd)
    print(f"[INFO] 点云已保存到: {filename}")
