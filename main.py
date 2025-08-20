#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
# 兼容 numpy 旧接口
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int

import os, time, argparse
import open3d as o3d 
from scipy.spatial.transform import Rotation as R 
from gsnet import AnyGrasp
from config.camera_params import tcp_T_camera
from camera.realsense_utils import get_realsense_pipeline
from grasp.move_utils import move_along_tcp
from grasp.grasp_loop import get_grasp_once
from grasp.grasp_pose import compute_grasp_pose
from grasp.grasp_executor import move_to_grasp
from robot.xarm_utils import init_xarm


def parse_args():
    parser = argparse.ArgumentParser(description="AnyGrasp XArm6 Auto Grasp")
    parser.add_argument('--checkpoint_path', required=True, help='模型权重路径')
    parser.add_argument('--max_gripper_width', type=float, default=0.1)
    parser.add_argument('--gripper_height', type=float, default=0.03)
    parser.add_argument('--top_down_grasp', action='store_true')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--save_ply', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    args.max_gripper_width = max(0.0, min(0.1, args.max_gripper_width))

    print("[INFO] 初始化 AnyGrasp...")
    anygrasp = AnyGrasp(args)
    anygrasp.load_net()

    print("[INFO] 启动 RealSense 相机...")
    pipeline, align, spatial, temporal, hole_filling = get_realsense_pipeline()
    time.sleep(3)

    print("[INFO] 初始化 XArm6...")
    arm = init_xarm()

    try:
        while True:
            top_grasp = get_grasp_once(
                pipeline, align, anygrasp,
                spatial=spatial, temporal=temporal, hole_filling=hole_filling,
                debug=True
            )
            if top_grasp is None:
                continue

            # 计算基座系位姿
            grasp_pos_m, grasp_rpy_deg = compute_grasp_pose(top_grasp, arm, tcp_T_camera)

            # 执行抓取
            success = move_to_grasp(grasp_pos_m, grasp_rpy_deg, arm, move_along_tcp)
            if success:
                print("[INFO] 抓取完成，继续下一轮")

    except KeyboardInterrupt:
        print("[INFO] 已终止")
    finally:
        pipeline.stop()
        arm.disconnect()


if __name__ == '__main__':
    main()
