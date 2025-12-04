from xarm.wrapper import XArmAPI

def init_xarm(ip='192.168.1.197'):
    arm = XArmAPI(ip)
    # 先检查是否连接成功
    if not arm.connected:
        raise Exception("XArm 连接失败，请检查 IP 和网络")
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_gripper_mode(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_speed(850)
    # arm.set_position(450,0,220,89,-89,91)
    arm.set_position(400,0,150,179,-45,1)
    arm.set_gripper_position(850)
    return arm
