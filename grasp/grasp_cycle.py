# grasp/grasp_cycle.py
import time

# 预设放置点 (mm & deg)
RELEASE_PLACE = [300, 360, 300, 180, 0, 0]
HOME_POS = [400, 0, 270, 180, 0, 0]

def pick_and_place(grasp_pos_m, grasp_rpy_deg, arm, move_along_tcp, debug=True):
    """
    执行完整的识别 -> 抓取 -> 放置 -> 回到初始点流程
    """
    print("\n[INFO] 开始抓取流程...")

    # 打开夹爪
    arm.set_gripper_position(850)

    # 移动到抓取点
    arm.set_position(
        x=(grasp_pos_m[0] * 1000),
        y=(grasp_pos_m[1] * 1000),
        z=(grasp_pos_m[2] * 1000),
        roll=grasp_rpy_deg[0],
        pitch=grasp_rpy_deg[1],
        yaw=grasp_rpy_deg[2],
        speed=150,
        wait=True
    )

    # 沿 TCP 下降 3.5cm
    pos, rpy = move_along_tcp(
        grasp_pos_m[0], grasp_pos_m[1], grasp_pos_m[2],
        grasp_rpy_deg[0], grasp_rpy_deg[1], grasp_rpy_deg[2],
        distance=0.035,
        axis='z',
        arm=None
    )
    arm.set_position(
        x=(pos[0] * 1000), y=(pos[1] * 1000), z=(pos[2] * 1000),
        roll=rpy[0], pitch=rpy[1], yaw=rpy[2],
        speed=30, wait=True
    )

    # 闭合夹爪
    arm.set_gripper_position(500)
    time.sleep(2)

    print("[INFO] 物体已抓取，前往放置点...")
    arm.set_position(x=(pos[0] * 1000), y=(pos[1] * 1000),z=300,roll=180,pitch=0,yaw=0,speed=150,wait=True)

    # 移动到放置点
    arm.set_position(*RELEASE_PLACE, speed=150, wait=True)

    # 打开夹爪，释放物体
    arm.set_gripper_position(850)
    time.sleep(1)

    print("[INFO] 物体已放置，返回初始点...")

    # 回到初始点
    arm.set_position(*HOME_POS, speed=150, wait=True)

    print("[INFO] 回到初始点，准备继续下一轮。")
    return True
