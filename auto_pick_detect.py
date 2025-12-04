#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_pick_detect_demo.py

演示：在 AnyGrasp + RealSense + XArm6 项目基础上增加 "目标检测 -> 提取点云子集 -> AnyGrasp 局部抓取" 的流程。

用法示例：
    python3 auto_pick_detect_demo.py --checkpoint_path ./model/checkpoint_detection.tar --detector ./weights/yolov5s.onnx --debug

参数:
    --detector PATH    可选：检测器模型路径（支持 .onnx 优先，若为 .pt 则尝试 torch.hub）
    --backend BACKEND  可选：onnx | torch | auto (auto = 优先 onnx，再尝试 torch)
    --min_points N     最少点数阈值（小的 mask 会被忽略）
    --debug            打开可视化
"""
import argparse
import time
import os
import numpy as np

# numpy 兼容旧接口
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int

# 延迟导入重量级模块以便快速失败回退
try:
    import cv2
except Exception as e:
    raise RuntimeError("请先安装 OpenCV (pip install opencv-python). 错误: " + str(e))

# 导入项目内部模块（假定这些文件已存在并与之前讨论的一致）
from gsnet import AnyGrasp
from camera.realsense_utils import get_realsense_pipeline, capture_aligned_frames
from grasp.grasp_pose import compute_grasp_pose
from grasp.move_utils import move_along_tcp
from robot.xarm_utils import init_xarm
from config.camera_params import fx, fy, cx, cy, workspace_lims, tcp_T_camera
import open3d as o3d

# 可选执行器：pick_and_place 优先，否则退回 move_to_grasp
try:
    from grasp.grasp_cycle import pick_and_place
    has_pick_and_place = True
except Exception:
    try:
        from grasp.grasp_executor import move_to_grasp
        has_pick_and_place = False
    except Exception:
        move_to_grasp = None
        has_pick_and_place = False


# ---------------------------
# Detector wrapper (flexible)
# ---------------------------
class SimpleColorSegmenter:
    """回退分割器：基于简单的亮度/背景差/颜色阈值来得到前景 mask。
       这是一个极简的策略，仅为演示和当检测器不可用时使用。
    """
    def __init__(self):
        pass

    def detect(self, image_bgr):
        # 返回 list of detections: { 'bbox':(x1,y1,x2,y2),'mask':mask,'score':1.0,'label':'unknown' }
        # 简易前景提取：高通 + morphological
        img_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img_gray, (5,5), 0)
        # 自适应阈值
        th = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY_INV,15,8)
        # 形态学去噪
        kernel = np.ones((5,5), np.uint8)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel, iterations=1)
        # 找连通域
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        dets = []
        h, w = th.shape
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # 忽略小区域
                continue
            x, y, ww, hh = cv2.boundingRect(cnt)
            mask = np.zeros((h, w), dtype=np.uint8)
            cv2.drawContours(mask, [cnt], -1, 255, -1)
            dets.append({'bbox': (x, y, x+ww, y+hh), 'mask': mask, 'score': float(area)/ (w*h), 'label': 'unknown'})
        # 若没有检测到，尝试用 Otsu 全图
        if len(dets) == 0:
            _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            kernel = np.ones((7,7), np.uint8)
            otsu = cv2.morphologyEx(otsu, cv2.MORPH_OPEN, kernel, iterations=1)
            contours, _ = cv2.findContours(otsu, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 1000: continue
                x, y, ww, hh = cv2.boundingRect(cnt)
                mask = np.zeros((h, w), dtype=np.uint8)
                cv2.drawContours(mask, [cnt], -1, 255, -1)
                dets.append({'bbox': (x, y, x+ww, y+hh), 'mask': mask, 'score': 0.5, 'label': 'unknown'})
        return dets


class DetectorWrapper:
    """
    支持灵活后端的检测器封装器：
      - ONNXRuntime（当检测器路径以.onnx结尾且已安装onnxruntime时推荐使用）
      - torch hub（ultralytics）当后端为'torch'且可用时（若环境缺少torch可能失败）
      - 默认回退至SimpleColorSegmenter
    该封装器提供 detect(image_bgr) -> 检测结果列表 {bbox,mask,score,label}
    """
    def __init__(self, model_path=None, backend='auto', conf_thres=0.3):
        self.model_path = model_path
        self.backend = backend
        self.conf_thres = conf_thres
        self.model = None
        self.mode = 'none'

        if model_path is None:
            print("[DETECT] no detector model provided, using simple color segmenter fallback.")
            self.mode = 'simple'
            self.model = SimpleColorSegmenter()
            return

        ext = os.path.splitext(model_path)[1].lower()
        # try ONNXRuntime if ext==.onnx or backend==onnx
        if (ext == '.onnx') or (backend == 'onnx') or backend == 'auto':
            try:
                import onnxruntime as ort
                self.ort = ort
                sess = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
                self.model = sess
                self.mode = 'onnx'
                print(f"[DETECT] Loaded ONNX model: {model_path}")
                return
            except Exception as e:
                print(f"[DETECT] ONNX load failed: {e}")

        # try torch hub (ultralytics)
        if (ext in ('.pt', '.pth')) or backend in ('torch', 'auto'):
            try:
                import torch
                # attempt to load custom model via ultralytics hub
                # Note: this will attempt to download ultralytics if not cached. May raise if environment blocks.
                model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=False)
                model.conf = conf_thres
                self.model = model
                self.mode = 'torch'
                print(f"[DETECT] Loaded torch detector (ultralytics): {model_path}")
                return
            except Exception as e:
                print(f"[DETECT] torch/ultralytics load failed: {e}")

        # fallback
        print("[DETECT] All detector backends failed — using simple color segmenter fallback.")
        self.mode = 'simple'
        self.model = SimpleColorSegmenter()

    def detect(self, image_bgr):
        """Return list of detections: {bbox, mask, score, label}"""
        if self.mode == 'onnx':
            return self._detect_onnx(image_bgr)
        elif self.mode == 'torch':
            return self._detect_torch(image_bgr)
        elif self.mode == 'simple':
            return self.model.detect(image_bgr)
        else:
            return []

    def _detect_torch(self, image_bgr):
        import torch
        results = self.model(image_bgr[..., ::-1])  # to RGB
        dets = []
        # results.xyxy[0] columns: x1,y1,x2,y2,conf,class
        try:
            arr = results.xyxy[0].cpu().numpy()
            for row in arr:
                x1, y1, x2, y2, conf, cls = row
                if conf < self.conf_thres: continue
                x1,y1,x2,y2 = map(int, [x1,y1,x2,y2])
                h, w = image_bgr.shape[:2]
                mask = np.zeros((h,w), np.uint8)
                # no mask from xyxy: leave mask zeros and rely on bbox
                dets.append({'bbox':(x1,y1,x2,y2),'mask':mask,'score':float(conf),'label':str(int(cls))})
        except Exception as e:
            print("[DETECT][torch] parse failed:", e)
        return dets

    def _detect_onnx(self, image_bgr):
        # Generic ONNX object detection (no unified postprocess). Try YOLOv5-like output first.
        try:
            import onnxruntime as ort
            sess = self.model
            h, w = image_bgr.shape[:2]
            # Preprocess: resize 640 maintaining ratio -> letterbox
            inp_h = inp_w = 640
            img = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            r = min(inp_w / w, inp_h / h)
            nw, nh = int(w * r), int(h * r)
            img_resized = cv2.resize(img, (nw, nh))
            canvas = np.full((inp_h, inp_w, 3), 114, dtype=np.uint8)
            x0 = (inp_w - nw) // 2
            y0 = (inp_h - nh) // 2
            canvas[y0:y0+nh, x0:x0+nw] = img_resized
            img_in = canvas.astype(np.float32) / 255.0
            img_in = np.transpose(img_in, (2,0,1))[None,...].astype(np.float32)
            ort_inputs = {sess.get_inputs()[0].name: img_in}
            ort_outs = sess.run(None, ort_inputs)
            # Try common YOLOv5-ish output format detection: (N,85) or (N,6)
            out = ort_outs[0]
            dets = []
            # Flatten predictions
            if out.ndim == 3:
                out = out[0]
            # out rows: x,y,w,h,obj_conf, class_conf...
            # We'll do a simple parse: find rows with conf > threshold
            if out.shape[1] >= 6:
                # decode boxes (yolo raw) or boxes directly
                for row in out:
                    conf = float(row[4])
                    if conf < self.conf_thres: continue
                    # x center, y center, w, h
                    cx, cy, bw, bh = row[0:4]
                    # map back to original image size
                    x1 = int((cx - bw/2 - x0) / r)
                    y1 = int((cy - bh/2 - y0) / r)
                    x2 = int((cx + bw/2 - x0) / r)
                    y2 = int((cy + bh/2 - y0) / r)
                    x1 = max(0, min(w-1, x1)); x2 = max(0, min(w-1, x2))
                    y1 = max(0, min(h-1, y1)); y2 = max(0, min(h-1, y2))
                    dets.append({'bbox':(x1,y1,x2,y2),'mask': np.zeros((h,w),np.uint8),'score':conf,'label':'unknown'})
            # If unable to parse, return empty to allow fallback higher-level
            return dets
        except Exception as e:
            print("[DETECT][onnx] error:", e)
            return []

# ---------------------------
# Helper: project mask -> points/colors
# ---------------------------
def mask_to_pointcloud(mask, depth, color, fx, fy, cx, cy, depth_scale=1000.0, z_min=0.02, z_max=1.5):
    """
    mask: uint8 (H,W) 0/255
    depth: depth image raw (H,W) in depth units (e.g. mm or uint16)
    color: BGR image (H,W,3)
    returns: points (N,3) in meters, colors (N,3) float32 0..1
    """
    h, w = mask.shape
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return np.zeros((0,3),dtype=np.float32), np.zeros((0,3),dtype=np.float32)
    z = depth[ys, xs].astype(np.float32) / depth_scale
    valid = (z > z_min) & (z < z_max)
    if not np.any(valid):
        return np.zeros((0,3),dtype=np.float32), np.zeros((0,3),dtype=np.float32)
    xs = xs[valid]; ys = ys[valid]; z = z[valid]
    x = (xs - cx) / fx * z
    y = (ys - cy) / fy * z
    pts = np.stack([x, y, z], axis=-1).astype(np.float32)
    cols = color[ys, xs].astype(np.float32) / 255.0
    return pts, cols

# ---------------------------
# Main loop
# ---------------------------
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint_path', required=True)
    p.add_argument('--detector', default=None, help='path to detector model (onnx or .pt)')
    p.add_argument('--backend', default='auto', choices=['auto','onnx','torch','simple'])
    p.add_argument('--min_points', type=int, default=500, help='min points inside mask to accept')
    p.add_argument('--debug', action='store_true')
    p.add_argument('--max_picks', type=int, default=0, help='0 infinite')
    p.add_argument('--max_gripper_width', type=float, default=0.085, help='机械爪最大张开宽度 (m)')
    p.add_argument('--gripper_depth', type=float, default=0.02, help='机械爪手指深度 (m)')
    p.add_argument('--gripper_height', type=float, default=0.03)
    p.add_argument('--top_down_grasp', action='store_true')
    return p.parse_args()

def run():
    args = parse_args()

    print("[INFO] 初始化 AnyGrasp...")
    anygrasp = AnyGrasp(args)
    anygrasp.load_net()

    print("[INFO] 初始化检测器...")
    detector = DetectorWrapper(model_path=args.detector, backend=args.backend)

    print("[INFO] 启动 RealSense 相机...")
    pack = get_realsense_pipeline()
    # get_realsense_pipeline 可能返回 pipeline,align 或 pipeline,align,spatial,temporal,hole_filling
    if isinstance(pack, (list,tuple)) and len(pack) == 5:
        pipeline, align, spatial, temporal, hole_filling = pack
    else:
        pipeline, align = pack
        spatial = temporal = hole_filling = None
    time.sleep(2.0)

    print("[INFO] 初始化 XArm6...")
    arm = init_xarm()

    pick_count = 0
    try:
        while (args.max_picks <= 0) or (pick_count < args.max_picks):
            color, depth = capture_aligned_frames(pipeline, align, spatial, temporal, hole_filling)
            if color is None or depth is None:
                print("[WARN] 未采到帧，重试...")
                time.sleep(0.2)
                continue

            # 检测（2D） -> 得到 bbox / mask 列表
            dets = detector.detect(color)
            if dets is None or len(dets) == 0:
                print("[INFO] 未检测到目标，尝试简单分割回退")
                # 即便 detector.mode == simple 也会返回非空; 这里防护
                dets = DetectorWrapper(model_path=None).detect(color)

            # 选择一个检测（策略：最高 score）
            dets_sorted = sorted(dets, key=lambda d: d.get('score',0.0), reverse=True)
            sel = dets_sorted[0]
            bbox = sel.get('bbox', None)
            mask = sel.get('mask', None)
            label = sel.get('label', 'unknown')
            score = sel.get('score', 0.0)

            # 如果没有 mask，仅用 bbox 生成 mask
            h, w = color.shape[:2]
            if mask is None or mask.sum() == 0:
                if bbox:
                    x1,y1,x2,y2 = bbox
                    mask = np.zeros((h,w), np.uint8)
                    x1c, y1c = max(0,x1), max(0,y1)
                    x2c, y2c = min(w-1,x2), min(h-1,y2)
                    mask[y1c:y2c, x1c:x2c] = 255
                else:
                    print("[WARN] 检测结果无 bbox 与 mask，跳过本帧")
                    time.sleep(0.1)
                    continue

            # 投影 mask -> points/colors
            pts, cols = mask_to_pointcloud(mask, depth, color, fx, fy, cx, cy)
            print(f"[INFO] mask 投影得到点数量: {pts.shape[0]} (score={score:.3f}, label={label})")
            if pts.shape[0] < args.min_points:
                print("[WARN] 点数量过少，跳过")
                time.sleep(0.1)
                continue

            # 将该物体点云喂给 AnyGrasp，让 AnyGrasp 只在该物体上寻找抓取位姿
            gg, cloud = anygrasp.get_grasp(pts, cols, lims=workspace_lims,
                                          apply_object_mask=True, collision_detection=True)

            if gg is None or len(gg) == 0:
                print("[INFO] AnyGrasp 未返回抓取候选，重试")
                time.sleep(0.1)
                continue

            gg = gg.nms().sort_by_score()
            best = gg[0:1]
            # 坐标系变换（Y、Z 轴取反）
            trans_mat = np.array([
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1]
            ], dtype=np.float64)
            # cloud_copy = None
            # if cloud is not None:
            #     try:
            #         if hasattr(cloud, 'points'):
            #             # cloud 是 Open3D 点云
            #             cloud_copy = o3d.geometry.PointCloud()
            #             cloud_copy.points = o3d.utility.Vector3dVector(np.asarray(cloud.points))
            #         else:
            #             # cloud 是 numpy array (N,3)
            #             cloud_copy = o3d.geometry.PointCloud()
            #             cloud_copy.points = o3d.utility.Vector3dVector(cloud)
            #         cloud_copy.transform(trans_mat)
            #     except Exception as e:
            #         print(f"[WARN] cloud transform 失败: {e}")
            #         cloud_copy = None

            # 7. 处理 gripper
            grippers_transformed = []
            if hasattr(best, "to_open3d_geometry_list"):
                grippers = best.to_open3d_geometry_list()
                for g in grippers:
                    g.transform(trans_mat)
                    grippers_transformed.append(g)

            # 8. 调试显示
            if cloud is not None:
                axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0,0,0])
                o3d.visualization.draw_geometries([cloud, *grippers_transformed, axis])

            # compute_grasp_pose expects a Grasp object (we pass best), arm and tcp_T_camera
            try:
                grasp_pos_m, grasp_rpy_deg = compute_grasp_pose(best, arm, tcp_T_camera)
            except Exception as e:
                print("[ERR] 计算抓取位姿失败：", e)
                time.sleep(0.1)
                continue

            # 执行抓取动作：优先 pick_and_place，否则 fallback move_to_grasp
            try:
                success = False
                if has_pick_and_place:
                    success = pick_and_place(grasp_pos_m, grasp_rpy_deg, arm, move_along_tcp)
                elif move_to_grasp is not None:
                    success = move_to_grasp(grasp_pos_m, grasp_rpy_deg, arm, move_along_tcp)
                else:
                    print("[ERR] 没有可用的抓取执行函数")
                    break
            except Exception as e:
                print("[ERR] 执行抓取失败：", e)
                success = False

            if success:
                pick_count += 1
                print(f"[INFO] 抓取成功 count={pick_count}, label={label}, score={score:.3f}")
            else:
                print("[WARN] 抓取未成功，继续")

            # short pause
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("[INFO] 用户中断，退出...")
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        try:
            arm.disconnect()
        except Exception:
            pass
        print("[INFO] 已退出")


if __name__ == '__main__':
    run()


#python3 auto_pick_detect.py --checkpoint_path ./model/checkpoint_detection.tar --detector ./weights/yolov5s.onnx --debug