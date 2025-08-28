#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import argparse
import numpy as np

# 兼容 numpy 旧接口
if not hasattr(np, 'float'):
    np.float = float
if not hasattr(np, 'int'):
    np.int = int

from gsnet import AnyGrasp
# 项目内模块
from camera.realsense_utils import get_realsense_pipeline
from grasp.grasp_loop import get_grasp_once
from grasp.grasp_pose import compute_grasp_pose
from grasp.grasp_cycle import pick_and_place
from grasp.move_utils import move_along_tcp
from robot.xarm_utils import init_xarm
from config.camera_params import tcp_T_camera


def parse_args():
    parser = argparse.ArgumentParser(description="AnyGrasp XArm6 连续抓取")
    parser.add_argument('--checkpoint_path', required=True, help='模型权重路径')
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


def run():
    args = parse_args()
    args.max_gripper_width = max(0.0, min(0.1, args.max_gripper_width))

    print("[INFO] 初始化 AnyGrasp…")
    anygrasp = AnyGrasp(args)
    anygrasp.load_net()

    print("[INFO] 启动 RealSense 相机…")
    pipeline, align, spatial, temporal, hole_filling = unpack_realsense_pipeline()
    time.sleep(2.0)  # 等待流稳定

    print("[INFO] 初始化 XArm6…")
    arm = init_xarm()
    fail_streak = 0
    pick_count = 0

    try:
        while (args.max_picks <= 0) or (pick_count < args.max_picks):

            # 1) 一次抓取检测（带滤波器可选）
            top_grasp = get_grasp_once(
                pipeline, align, anygrasp,
                spatial=spatial, temporal=temporal, hole_filling=hole_filling,
                debug=args.debug
            )

            if top_grasp is None:
                fail_streak += 1
                print(f"[WARN] 未检测到抓取点，连续失败 {fail_streak} 次")
                # 达到阈值，回零位稳态观察
                if args.rehoming_interval > 0 and fail_streak % args.rehoming_interval == 0:
                    try:
                        arm.set_position(400, 0, 270, 180, 0, 0, speed=150, wait=True)
                        print("[INFO] 连续失败，回到初始位姿观察…")
                    except Exception as e:
                        print(f"[WARN] 回初始位姿失败：{e}")
                continue

            # 2) 计算基座系位姿
            try:
                grasp_pos_m, grasp_rpy_deg = compute_grasp_pose(top_grasp, arm, tcp_T_camera)
            except Exception as e:
                fail_streak += 1
                print(f"[ERR] 计算抓取位姿失败：{e}")
                continue

            # 3) 执行抓取（无用户确认，直接动作）
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


#python3 auto_pick.py --checkpoint_path ./model/checkpoint_detection.tar