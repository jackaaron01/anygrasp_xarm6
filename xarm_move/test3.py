import time
import json
import os
from datetime import datetime
from xarm.wrapper import XArmAPI


# ====================== 日志系统 ======================

def log(msg, level="INFO", save_file=True):
    """
    简易日志输出函数
    """
    t = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    text = f"[{t}] [{level}] {msg}"
    print(text)
    if save_file:
        with open("arm_log.txt", "a", encoding="utf-8") as f:
            f.write(text + "\n")


# ====================== 偏移校准模块 ======================

OFFSET_FILE = "offset.json"


def load_offset():
    if os.path.exists(OFFSET_FILE):
        with open(OFFSET_FILE, "r") as f:
            return json.load(f)
    return {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}


def save_offset(offset):
    with open(OFFSET_FILE, "w") as f:
        json.dump(offset, f, indent=2)
    log("✅ 偏移补偿已保存至 offset.json", "INFO")


def apply_offset(pos, offset):
    """应用偏移修正"""
    for k in offset:
        pos[k] += offset[k]
    return pos


# ====================== 错误与警告回调 ======================

def handle_err_warn_changed(item):
    log(f"ErrorCode: {item['error_code']}, WarnCode: {item['warn_code']}", "WARN")


# ====================== 初始化函数 ======================

def init_arm(ip: str):
    log(f"尝试连接机械臂 {ip} ...")

    arm = XArmAPI(ip)
    arm.register_error_warn_changed_callback(handle_err_warn_changed)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_gripper_mode(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_position(800, wait=True)

    angles = arm.get_servo_angle(is_radian=False)
    pos = arm.get_position()

    log("机械臂初始化完成")
    log(f"当前位置: {pos}")
    log(f"当前关节角度: {angles}")

    return arm


# ====================== 校准与检查 ======================

def check_joint_angles(arm, tolerance=1.0):
    """
    检查关节角是否偏离启动基准。
    """
    ret, base_angles = arm.get_servo_angle(is_radian=False)
    if ret != 0 or not base_angles:
        log("无法获取关节角信息", "WARN")
        return
    log("关节角状态检查：")
    for i, ang in enumerate(base_angles):
        if abs(ang) > 360:
            log(f"⚠️ 第{i+1}关节角异常：{ang}° 超出范围", "WARN")
        elif abs(ang) > tolerance:
            log(f"关节{i+1}: {ang:.2f}°", "DEBUG")
    log("✅ 关节角检查完成")


# ====================== 动作函数 ======================

def arm_move(arm):
    """测试动作序列"""
    # 操作点参数
    op_pos = {'x': 520, 'y': -20, 'z': 60, 'roll': -89, 'pitch': -89, 'yaw': -89}

    # 下移至操作点
    arm.set_position(**op_pos, speed=100, mvacc=20)
    arm.set_gripper_position(250, wait=True)

    # 上移
    arm.set_position(x=520, y=-20, z=160, roll=-89, pitch=-89, yaw=-89, speed=100, mvacc=20)

    # 移动目标点1
    arm.set_position(x=145, y=-320, z=110, roll=-89, pitch=-89, yaw=-89, speed=100, mvacc=20)
    # 移动目标点2
    arm.set_position(x=145, y=-480, z=110, roll=-89, pitch=-89, yaw=-89, speed=100, mvacc=20)
    time.sleep(2)
    # 返回目标点1
    arm.set_position(x=145, y=-320, z=110, roll=-89, pitch=-89, yaw=-89, speed=100, mvacc=20)

    # 回原位
    arm.set_position(**op_pos, speed=100, mvacc=20)
    arm.set_gripper_position(800, wait=True)
    log("✅ 动作序列执行完毕")


def arm_smart_move(arm):
    """智能启动逻辑 + 校准补偿"""
    target_pos = {'x': 520, 'y': -20, 'z': 60, 'roll': 89, 'pitch': -89, 'yaw': 91}
    safe_pos = {'x': 520, 'y': -20, 'z': 160, 'roll': 89, 'pitch': -89, 'yaw': 91}

    result = arm.get_position()
    if not result or result[0] != 0 or not result[1]:
        log("无法获取机械臂位置", "ERROR")
        return
    current_pos = result[1]
    cx, cy, cz, cr, cp, cyaw = current_pos[:6]
    tolerance = 5

    # 自动校准补偿加载
    offset = load_offset()
    log(f"读取到偏移补偿: {offset}")

    # 检查是否在目标点附近
    in_position = (
        abs(cx - target_pos['x']) < tolerance and
        abs(cy - target_pos['y']) < tolerance and
        abs(cz - target_pos['z']) < tolerance
    )

    if in_position:
        log("✅ 当前位置接近目标，执行动作。")
        arm_move(arm)
    else:
        log("ℹ️ 检测到位置偏移，记录补偿并移动到安全点。")
        offset = {
            "x": target_pos["x"] - cx,
            "y": target_pos["y"] - cy,
            "z": target_pos["z"] - cz,
            "roll": target_pos["roll"] - cr,
            "pitch": target_pos["pitch"] - cp,
            "yaw": target_pos["yaw"] - cyaw
        }
        save_offset(offset)
        # 移动到安全位
        safe_pos = apply_offset(safe_pos, offset)
        arm.set_position(**safe_pos, speed=200, mvacc=50, wait=True)
        arm_move(arm)


# ====================== 主程序入口 ======================

if __name__ == "__main__":
    ARM_IP = "192.168.1.197"
    arm = init_arm(ARM_IP)

    check_joint_angles(arm)
    log("\n=== 开始智能任务执行 ===\n")
    arm_smart_move(arm)

    arm.disconnect()
    log("🔚 已断开机械臂连接")
