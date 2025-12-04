#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from xarm.wrapper import XArmAPI
import time

# ----------------------------------------
# 修改为你的机械臂 IP 地址
# ----------------------------------------
ROBOT_IP = "192.168.1.197"

def main():
    print("=== XArm 测试程序启动 ===")

    # 1. 连接机械臂
    print(f"正在连接 XArm6，IP: {ROBOT_IP} ...")
    arm = XArmAPI(ROBOT_IP)

    if not arm.connected:
        print("❌ 连接失败！请检查：")
        print(" - IP 地址是否正确")
        print(" - 电脑与机械臂是否在同一网段")
        print(" - 能否 ping 通机械臂")
        print(" - 急停是否松开")
        return
    
    print("✅ 已成功连接机械臂！")

    # 2. 清除错误
    arm.clean_error()
    arm.clean_warn()

    # 3. 上电
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    print("⚡ 机械臂已上电")
    # arm.set_position(450,0,220,89,-89,91)

    time.sleep(1)

    # 4. 获取当前位姿
    ret, pose = arm.get_position(is_radian=False)
    print("\n📌 当前机械臂末端位姿：")
    print(pose)

    # 5. 测试轻微移动（安全）
    print("\n➡️ 进行测试移动：向 X 方向移动 +20mm")

    target = pose.copy()
    target[0] += 20    # x +20mm

    ret = arm.set_position(*target, radius=20, speed=10, is_radian=False)
    if ret != 0:
        print("❌ 移动失败，错误码：", ret)
    else:
        print("✅ 移动完成")

    time.sleep(1)

    # 6. 移回原点位置
    print("\n↩️ 返回初始位置")
    arm.set_position(*pose, radius=20, speed=10, is_radian=False)

    time.sleep(1)

    print("\n=== ✔️ XArm 测试完成 ===")

    arm.disconnect()

if __name__ == '__main__':
    main()
