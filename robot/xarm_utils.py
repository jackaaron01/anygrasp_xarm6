from xarm.wrapper import XArmAPI

def init_xarm(ip='192.168.1.223'):
    arm = XArmAPI(ip)
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_gripper_mode(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_speed(850)
    arm.set_position(400,0,270,180,0,0)
    arm.set_gripper_position(850)
    return arm
