#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import argparse
import numpy as np
import cv2

# 兼容 numpy 旧接口
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int

from gsnet import AnyGrasp
# 项目内模块
from camera.realsense_utils import get_realsense_pipeline
from grasp.grasp_once import get_grasp_once
from grasp.grasp_pose import compute_grasp_pose
from grasp.grasp_cycle import pick_and_place
from grasp.move_utils import move_along_tcp
from robot.xarm_utils import init_xarm
from config.camera_params import tcp_T_camera
from vision.detector_utils import ObjectDetector


def parse_args():
    parser = argparse.ArgumentParser(description="AnyGrasp + YOLO XArm6 连续抓取")
    parser.add_argument('--checkpoint_path', required=True, help='AnyGrasp 模型权重路径')
    parser.add_argument('--detector', type=str, default=None,
                        help='YOLO 检测模型权重路径 (例如 yolov5s.pt)，不传则不启用 YOLO')
    parser.add_argument('--max_gripper_width', type=float, default=0.1)
    parser.add_argument('--gripper_height', type=float, default=0.03)
    parser.add_argument('--top_down_grasp', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--save_ply', action='store_true')
    # 连续抓取相关参数
    parser.add_argument('--max_picks', type=int, default=0,
                        help='最多抓取次数，0 为无限循环')
    parser.add_argument('--cycle_pause', type=float, default=0.5,
                        help='每次抓取完成后的暂停秒数')
    parser.add_argument('--rehoming_interval', type=int, default=5,
                        help='连续失败多少次后回到初始位姿')
    return parser.parse_args()


def unpack_realsense_pipeline():
    pack = get_realsense_pipeline()
    if isinstance(pack, (list, tuple)) and len(pack) == 5:
        pipeline, align, spatial, temporal, hole_filling = pack
    else:
        pipeline, align = pack
        spatial = temporal = hole_filling = None
    return pipeline, align, spatial, temporal, hole_filling


import cv2
import numpy as np

def run_detector(detector, pipeline, align, args):
    """
    使用 YOLO/分割模型检测物体，返回:
      roi: (x1, y1, x2, y2) 或 None
      annotated: 可视化图像 (可 None)
      best: 最优检测结果字典 (bbox, conf, class, mask, area)
      color_image: 原始彩色帧
    """
    roi = None
    annotated = None
    best = None
    color_image = None

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    if not color_frame:
        print("[WARN] 未获取到彩色帧")
        return None, None, None, None

    color_image = np.asanyarray(color_frame.get_data())
    detections = detector.detect_objects(color_image, conf_thres=0.5)

    annotated = color_image.copy()

    if len(detections) == 0:
        print("[WARN] 检测器未检测到物体")
        return None, annotated, None, color_image

    # 选择目标策略（置信度 * sqrt(area)）
    def score_fn(d):
        return d.get("conf", 0.0) * (np.sqrt(max(1, d.get("area", 1))))

    best = max(detections, key=score_fn)

    # 如果有 mask
    if best.get("mask") is not None:
        mask = best["mask"].astype(np.uint8)
        ys, xs = np.where(mask > 0)
        if len(xs) > 0:
            x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
            roi = (x1, y1, x2, y2)
        else:
            x1, y1, x2, y2 = best["bbox"]
            roi = (x1, y1, x2, y2)

        # 可视化：mask 半透明叠加 + 轮廓
        color_mask = np.zeros_like(annotated)
        color_mask[mask > 0] = (0, 200, 0)
        annotated = cv2.addWeighted(annotated, 0.6, color_mask, 0.4, 0)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cv2.drawContours(annotated, contours, -1, (0, 255, 0), 2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(annotated, f"conf:{best['conf']:.2f}", (x1, max(0, y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    else:
        # 只有 bbox
        x1, y1, x2, y2 = best["bbox"]
        roi = (x1, y1, x2, y2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(annotated, f"conf:{best['conf']:.2f}", (x1, max(0, y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    # debug 模式显示窗口
    if args.debug:
        try:
            cv2.imshow("Detector (segmentation overlay)", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] 用户请求退出检测窗口")
                return None, annotated, None, color_image
        except Exception:
            pass

    print(f"[INFO] 检测到物体 ROI={roi}, conf={best.get('conf',0):.2f}")
    return roi, annotated, best, color_image



def run():
    args = parse_args()
    args.max_gripper_width = max(0.0, min(0.1, args.max_gripper_width))

    print("[INFO] 初始化 AnyGrasp…")
    anygrasp = AnyGrasp(args)
    anygrasp.load_net()

    # 初始化 YOLO 检测器
    detector = None
    if args.detector:
        try:
            print(f"[INFO] 加载检测模型: {args.detector}")
            detector = ObjectDetector(model_path=args.detector, device="cuda")
            print("[INFO] YOLO 检测器加载成功")
        except Exception as e:
            print(f"[ERR] YOLO 检测器加载失败：{e}")
            detector = None
    else:
        print("[INFO] 未启用 YOLO 检测器")

    print("[INFO] 启动 RealSense 相机…")
    pipeline, align, spatial, temporal, hole_filling = unpack_realsense_pipeline()
    time.sleep(2.0)  # 等待流稳定

    print("[INFO] 初始化 XArm6…")
    arm = init_xarm()
    fail_streak = 0
    pick_count = 0

    try:
        while (args.max_picks <= 0) or (pick_count < args.max_picks):

    # ========== 1) YOLO/Segmentation 检测桌面物体 ==========
            if detector:
                roi, annotated, best, color_image = run_detector(detector, pipeline, align, args)
                if roi is None:
                    fail_streak += 1
                    continue
            else:
                # detector 未启用，读取相机但不做检测
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    if args.debug:
                        cv2.imshow("Camera", color_image)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                print("[INFO] 未启用检测器，跳过检测阶段")


            # ========== 2) AnyGrasp 计算抓取 ==========
            top_grasp = get_grasp_once(
                pipeline, align, anygrasp,
                spatial=spatial, temporal=temporal, hole_filling=hole_filling,
                debug=args.debug,
                roi=roi  # 将检测框传给 AnyGrasp
            )

            if top_grasp is None:
                fail_streak += 1
                print(f"[WARN] 未检测到抓取点，连续失败 {fail_streak} 次")
                if args.rehoming_interval > 0 and fail_streak % args.rehoming_interval == 0:
                    try:
                        arm.set_position(400, 0, 270, 180, 0, 0, speed=150, wait=True)
                        print("[INFO] 连续失败，回到初始位姿观察…")
                    except Exception as e:
                        print(f"[WARN] 回初始位姿失败：{e}")
                continue

            # ========== 3) 计算位姿 ==========
            try:
                grasp_pos_m, grasp_rpy_deg = compute_grasp_pose(top_grasp, arm, tcp_T_camera)
            except Exception as e:
                fail_streak += 1
                print(f"[ERR] 计算抓取位姿失败：{e}")
                continue

            # ========== 4) 执行抓取动作 ==========
            ok = False
            try:
                ok = pick_and_place(grasp_pos_m, grasp_rpy_deg, arm, move_along_tcp)
            except Exception as e:
                print(f"[ERR] 执行抓取动作失败：{e}")

            if ok:
                pick_count += 1
                fail_streak = 0
                print(f"[INFO] ✅ 完成第 {pick_count} 次夹取")
                time.sleep(args.cycle_pause)
            else:
                fail_streak += 1
                print(f"[WARN] 抓取动作失败，连续失败 {fail_streak} 次")

    except KeyboardInterrupt:
        print("[INFO] 用户中断，准备退出…")
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        try:
            arm.disconnect()
        except Exception:
            pass
        print("[INFO] 已安全退出")


if __name__ == '__main__':
    run()
