# grasp/grasp_executor.py
import time

def move_to_grasp(grasp_pos_m, grasp_rpy_deg, arm, move_along_tcp, debug=True):
    """执行抓取动作"""
    print('\n[INFO] 在 Open3D 窗口中仅显示最高分抓取姿态。请检查并按回车确认是否执行抓取（或关闭窗口退出）。')
    ok = input('按 Enter 执行抓取，输入 n 并回车取消：')
    if ok.strip().lower() == 'n':
        arm.set_position(400, 0, 270, 180, 0, 0)
        print('[INFO] 用户取消，本轮重新采集')
        return False

    # 执行抓取流程
    arm.set_gripper_position(850)
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

    arm.set_gripper_position(500)
    time.sleep(3)

    # 回到初始位置
    arm.set_position(400, 0, 270, 180, 0, 0, speed=150)
    return True
