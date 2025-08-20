
## 1️⃣ 可调用函数总结

### **camera/realsense\_utils.py**

```python
def get_realsense_pipeline():
    """
    初始化 RealSense 管道和对齐对象，以及可选滤波器
    返回: pipeline, align, spatial_filter, temporal_filter, hole_filling_filter
    """
    ...
    
def capture_aligned_frames(pipeline, align, spatial=None, temporal=None, hole_filling=None):
    """
    获取对齐的 RGB-D 图像，并对深度图应用滤波器
    返回: color_image (H,W,3), depth_image (H,W)
    """
    ...
```

### **camera/pointcloud\_utils.py**

```python
def convert_to_pointcloud(color, depth, fx, fy, cx, cy, scale=1000.0, z_min=0.1, z_max=0.8):
    """
    将 RGB-D 图像转换为点云
    返回: points (N,3), colors (N,3)
    """
    
def save_pointcloud_to_ply(points, colors, filename):
    """
    保存点云到 PLY 文件
    """
```

### **grasp/grasp\_loop.py**

```python
def get_grasp_once(pipeline, align, anygrasp, spatial=None, temporal=None, hole_filling=None, debug=False):
    """
    获取一次抓取结果
    返回: top_grasp (AnyGrasp 格式) 或 None
    """
```

### **grasp/grasp\_pose.py**

```python
def compute_grasp_pose(top_grasp, arm, tcp_T_camera):
    """
    将抓取点从相机坐标系转换为机械臂基座坐标系
    返回: grasp_pos_m (m), grasp_rpy_deg (deg)
```

### **grasp/grasp\_executor.py**

```python
def move_to_grasp(grasp_pos_m, grasp_rpy_deg, arm, move_func):
    """
    执行抓取动作
    返回: success (True/False)
```

### **grasp/move\_utils.py**

```python
def move_along_tcp(arm, dx, dy, dz):
    """
    沿机械臂末端姿态方向移动
    """
```

### **robot/xarm\_utils.py**

```python
def init_xarm():
    """
    初始化 XArm6 机械臂，返回 arm 对象
    """
```

---

## 2️⃣ 虚拟环境依赖（Conda 环境推荐）

**核心依赖：**

* Python 3.9
* PyTorch >=1.7.1，CUDA 11.x / 12.1
* MinkowskiEngine 0.5.4
* open3d
* numpy
* scipy
* pyrealsense2
* Pillow (注意 PIL 相关问题，避免 Lerc.so 报错)
* gsnet (AnyGrasp SDK)


> ⚠️ 注意：
>
> * MinkowskiEngine 需要和 PyTorch CUDA 版本对应，否则无法编译。
> * 如果你在 Ubuntu 20.04 + CUDA 11.x，可以通过 `conda install pytorch torchvision torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia` 安装。

---

## 3️⃣ 项目结构

```
anygrasp_xarm6_project/
│
├─ model/                     # AnyGrasp 模型权重
├─ resource/                  # 示例数据
├─ camera/
│   ├─ realsense_utils.py
│   └─ pointcloud_utils.py
├─ grasp/
│   ├─ grasp_loop.py
│   ├─ grasp_pose.py
│   ├─ grasp_executor.py
│   └─ move_utils.py
├─ robot/
│   └─ xarm_utils.py
├─ config/
│   └─ camera_params.py
├─ main.py
├─ requirements.txt
└─ README.md
```

---

## 4️⃣ 项目简介

````markdown
# AnyGrasp + XArm6 自动抓取项目

## 功能简介
- 基于 Intel RealSense D435 相机和 AnyGrasp 模型进行桌面物体抓取。
- XArm6 机械臂执行抓取动作。
- 支持深度滤波器提升点云质量。
- 可调节抓取参数（抓手宽度、高度、是否Top-Down抓取）。

## 环境依赖
推荐使用 Conda 创建虚拟环境：
```bash
conda create -n anygrasp python=3.9
conda activate anygrasp
pip install -r requirements.txt
````

## 主要依赖

* PyTorch >= 1.7.1 (CUDA 11.x / 12.1)
* MinkowskiEngine==0.5.4
* open3d>=0.16
* pyrealsense2
* Pillow
* numpy, scipy

## 文件说明

* `camera/`：RealSense 相机接口及点云转换工具
* `grasp/`：抓取点计算及抓取执行逻辑
* `robot/`：XArm6 初始化及运动工具
* `config/`：相机内参、工作空间等参数
* `main.py`：主程序入口

## 使用方法

```bash
python3 main.py --checkpoint_path ./model/checkpoint_detection.tar --debug
```

可选参数：

* `--max_gripper_width`：最大抓手宽度，默认 0.1 m
* `--gripper_height`：抓手高度，默认 0.03 m
* `--top_down_grasp`：是否只使用垂直抓取
* `--debug`：调试模式，显示点云和抓手
* `--save_ply`：保存点云文件

## 注意事项

* 确保 CUDA 版本与 PyTorch 匹配。
* 如果 PIL 导致 Lerc.so 报错，请升级 `libstdc++` 或在 conda 中安装 `libstdcxx-ng`。
* 滤波器已内置，点云质量差可调整 `spatial`, `temporal`, `hole_filling` 参数。

torch>=1.7.1
torchvision
torchaudio
torchtext
torchsparse
MinkowskiEngine==0.5.4
open3d>=0.16
numpy>=1.21
scipy>=1.7
pyrealsense2>=2.50
Pillow>=9.0
matplotlib