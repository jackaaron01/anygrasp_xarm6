import time
from xarm.wrapper import XArmAPI

# ---------------- 配置常量（便于测试调整） ----------------
ARM_IP = "192.168.1.197"

# 参考位与安全位（字典形式）
REF_DOWN = {'x': 520, 'y': -20, 'z': 55,  'roll': -89, 'pitch': -89, 'yaw': -91}
REF_UP   = {'x': 520, 'y': -20, 'z': 110, 'roll': -89, 'pitch': -89, 'yaw': -91}
PLACE1   = {'x': 145, 'y': -320, 'z': 110, 'roll': 178, 'pitch': -88, 'yaw': -88}
PLACE2   = {'x': 140, 'y': -460, 'z': 115, 'roll': 178, 'pitch': -88, 'yaw': -88}

# 夹爪开合值
GRIP_OPEN = 800
GRIP_CLOSE = 250

# 默认运动参数
DEFAULT_SPEED = 150
DEFAULT_MVACC = 40

# 容差（用于位置判断）
DEFAULT_TOLERANCE_MM = 5.0
DEFAULT_TOLERANCE_DEG = 2.0

# get_position 重试次数/间隔
POS_RETRY = 3
POS_RETRY_DELAY = 0.1

# ---------------- 实用工具函数 ----------------
def handle_err_warn_changed(item):
    """错误/警告回调（测试阶段简单打印）"""
    print(f"[ERR_WARN] ErrorCode: {item['error_code']}, WarnCode: {item['warn_code']}")

def init_arm(ip: str):
    """初始化机械臂（连接、回调、启用电机、夹爪打开）"""
    arm = XArmAPI(ip)
    print(f"[INFO] 连接到机械臂: {ip}")

    arm.register_error_warn_changed_callback(handle_err_warn_changed)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)

    # 夹爪初始化
    arm.set_gripper_mode(0)
    arm.set_gripper_enable(True)
    arm.set_gripper_position(GRIP_OPEN, wait=True)

    # 打印状态（容错：某些 API 可能返回 None）
    try:
        angles = arm.get_servo_angle(is_radian=False)
    except Exception:
        angles = None
    pos = arm.get_position()
    print("[INFO] 初始化完成")
    print(f"  当前位置 raw: {pos}")
    print(f"  关节角度: {angles}")

    return arm

def get_cartesian_pose(arm, retries=POS_RETRY, delay=POS_RETRY_DELAY):
    """
    读取并解析 get_position() 返回值，返回 dict 或 None。
    XArmAPI.get_position() 通常返回 (code, [x,y,z,roll,pitch,yaw])
    """
    for _ in range(retries):
        res = arm.get_position()
        if res and isinstance(res, (list, tuple)) and len(res) >= 2 and res[0] == 0 and res[1]:
            arr = res[1]
            if len(arr) >= 6:
                return {'x': arr[0], 'y': arr[1], 'z': arr[2],
                        'roll': arr[3], 'pitch': arr[4], 'yaw': arr[5]}
        time.sleep(delay)
    return None

def move_pose(arm, pose: dict, speed=DEFAULT_SPEED, mvacc=DEFAULT_MVACC, wait=True):
    """统一的移动接口（使用字典作为 pose）"""
    arm.set_position(x=pose['x'], y=pose['y'], z=pose['z'],
                     roll=pose['roll'], pitch=pose['pitch'], yaw=pose['yaw'],
                     speed=speed, mvacc=mvacc, wait=wait)

def is_pose_close(p1: dict, p2: dict, tol_mm=DEFAULT_TOLERANCE_MM, tol_deg=DEFAULT_TOLERANCE_DEG):
    """判断两个位姿在容差内（位置以 mm，角度以度）"""
    if p1 is None or p2 is None:
        return False
    return (abs(p1['x'] - p2['x']) <= tol_mm and
            abs(p1['y'] - p2['y']) <= tol_mm and
            abs(p1['z'] - p2['z']) <= tol_mm and
            abs(p1['roll'] - p2['roll']) <= tol_deg and
            abs(p1['pitch'] - p2['pitch']) <= tol_deg and
            abs(p1['yaw'] - p2['yaw']) <= tol_deg)

# ---------------- 任务函数 ----------------
def arm_move(arm):
    """执行一次 pick & place（使用封装的 move_pose）"""
    # 从安全位慢速下降并夹取
    move_pose(arm, REF_DOWN, speed=80, mvacc=20, wait=True)
    arm.set_gripper_position(GRIP_CLOSE, wait=True)

    # 抬起并前往放置点
    move_pose(arm, REF_UP, speed=120, mvacc=30, wait=True)
    move_pose(arm, PLACE1, speed=120, mvacc=30, wait=True)
    move_pose(arm, PLACE2, speed=120, mvacc=30, wait=True)

    time.sleep(5)  # 放置等待（测试阶段可保留）
    move_pose(arm, PLACE1, speed=120, mvacc=30, wait=True)

    # 回到工作位并释放
    move_pose(arm, REF_DOWN, speed=80, mvacc=20, wait=True)
    arm.set_gripper_position(GRIP_OPEN, wait=True)

def arm_smart_move(arm, tolerance_mm=DEFAULT_TOLERANCE_MM, tolerance_deg=DEFAULT_TOLERANCE_DEG):
    """
    智能判断并执行：
      - 若已在 REF_DOWN 附近 -> 直接执行 arm_move
      - 否则先到 REF_UP（安全高位）再执行
    """
    cur = get_cartesian_pose(arm)
    print(f"[INFO] 读取当前位姿: {cur}")

    if cur and is_pose_close(cur, REF_DOWN, tol_mm=tolerance_mm, tol_deg=tolerance_deg):
        print("[INFO] 已位于参考下降位置，直接执行任务")
        arm_move(arm)
    else:
        print("[INFO] 不在参考位置，先移动到安全高位再执行")
        move_pose(arm, REF_UP, speed=200, mvacc=50, wait=True)
        arm_move(arm)

# ---------------- 主入口 ----------------
if __name__ == "__main__":
    arm = None
    try:
        arm = init_arm(ARM_IP)
        print("\n=== 开始智能任务执行 ===")
        arm_smart_move(arm)
    except Exception as e:
        print(f"[ERROR] 运行中出现异常: {e}")
    finally:
        if arm:
            try:
                arm.disconnect()
            except Exception:
                pass
        print("🔚 已断开连接（或尝试断开）")
