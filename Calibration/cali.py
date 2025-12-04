#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Eye-in-hand hand–eye calibration for xArm6 + Intel RealSense (color stream)

What this script does
---------------------
- Connects to an xArm6 via xArm Python SDK.
- Grabs RGB frames from a RealSense camera and detects a chessboard.
- Collects paired poses: base->gripper (from robot) and target->camera (from solvePnP).
- Runs OpenCV calibrateHandEye (Tsai by default) to compute gripper->camera (ee_T_cam).
- Saves results (NPZ + YAML) and provides a simple verification routine.

Usage example
-------------
python handeye_xarm6_realsense.py \
  --robot-ip 192.168.1.223 \
  --rows 5 --cols 8 --square 0.025 \
  --samples 15 --method tsai --preview

Hotkeys during capture window
-----------------------------
[Enter/Space]  capture a sample
[v]            run quick verification with a fresh view (optional)
[s]            save raw samples to disk
[q/ESC]        finish collection and calibrate

Notes
-----
- Keep the chessboard rigidly fixed in the workspace (mounted to table or a stand).
- Move the robot wrist through diverse *orientations* (rotate around all axes),
  including in-place wrist rotation, not just translations. Aim for 10–20 good poses.
- Default units: robot in *meters*, chessboard square_size in *meters*.
- Result ee_T_cam maps camera frame to the gripper frame (OpenCV returns cam->gripper).
  To compute base->camera: base_T_cam = base_T_ee @ ee_T_cam.

Dependencies
------------
- xArm-Python-SDK (pip install xarm)
- pyrealsense2 (Intel RealSense SDK Python bindings)
- opencv-python
- numpy, scipy
- (optional) open3d for 3D visualization (not required)
"""

import os
import sys
import time
import argparse
import json
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# --- Robot wrapper (xArm6) ---------------------------------------------------

def euler_xyz_deg_to_rot(roll_deg, pitch_deg, yaw_deg):
    return R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()

class XArm6Controller:
    def __init__(self, robot_ip, verbose: bool = True):
        try:
            from xarm.wrapper import XArmAPI
        except Exception as e:
            raise RuntimeError("xArm-Python-SDK not found. Install with `pip install xarm`.\n" + str(e))
        self.arm = XArmAPI(robot_ip)
        if verbose:
            print(f"Connecting to xArm at {robot_ip} ...")
        code = self.arm.connect()
        # if code != 0:
        #     raise RuntimeError(f"Failed to connect to xArm, code={code}")
        self.arm.motion_enable(True)
        self.arm.set_mode(0)   # position mode
        self.arm.set_state(0)  # ready
        if verbose:
            print("xArm connected and enabled.")

    def get_base_T_gripper(self):
        """Return 4x4 homogeneous transform of gripper(TCP) in base frame.
        Units converted to meters. Orientation from roll-pitch-yaw (deg, xyz order)."""
        code, pose = self.arm.get_position(is_radian=False)
        if code != 0 or pose is None:
            raise RuntimeError(f"get_position failed, code={code}")
        x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg = pose[:6]
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = euler_xyz_deg_to_rot(roll_deg, pitch_deg, yaw_deg)
        T[:3, 3] = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0
        return T

    def disconnect(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass

# --- RealSense wrapper --------------------------------------------------------

class RealSenseCamera:
    def __init__(self, use_color=True):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.use_color = use_color
        if use_color:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # depth is not required for chessboard solvePnP
        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # lower log noise
        try:
            rs.log_to_console(rs.log_severity.error)
        except Exception:
            pass
        
        self.profile = None
        self.intrinsics = None

    def start(self):
        self.profile = self.pipeline.start(self.config)
        time.sleep(0.5)
        self._fetch_intrinsics()

    def stop(self):
        self.pipeline.stop()

    def _fetch_intrinsics(self):
        color_profile = self.profile.get_stream(self.rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        K = np.array([[intr.fx,      0.0,     intr.ppx],
                      [0.0,      intr.fy,     intr.ppy],
                      [0.0,          0.0,        1.0 ]], dtype=np.float64)
        dist = np.array(intr.coeffs[:5], dtype=np.float64)  # k1, k2, p1, p2, k3
        self.intrinsics = (K, dist, intr.width, intr.height)

    def get_color(self, timeout_s=2.0):
        """Return BGR image as numpy array."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if color_frame:
                img = np.asanyarray(color_frame.get_data())
                return img
        return None

# --- Chessboard detection & solvePnP -----------------------------------------

def detect_chessboard_T_target2cam(bgr, rows, cols, square_size_m, K, dist, draw=True):
    """Find chessboard corners and solvePnP.
    Returns: 4x4 T_target2cam or None.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    pattern_size = (cols, rows)  # OpenCV expects (columns, rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return None, bgr

    # Refine corners
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)

    # Prepare object points in target frame (Z=0 plane)
    objp = np.zeros((rows * cols, 3), np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] *= float(square_size_m)

    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, bgr

    R_tc, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_tc
    T[:3, 3] = tvec.reshape(3)

    if draw:
        vis = cv2.drawChessboardCorners(bgr.copy(), pattern_size, corners, found)
        axis = square_size_m * 3.0
        # draw coordinate axes on the board
        corners_reshaped = corners.reshape(-1, 1, 2)
        imgpts, _ = cv2.projectPoints(np.float32([[0,0,0],[axis,0,0],[0,axis,0],[0,0,-axis]]), rvec, tvec, K, dist)
        p0 = tuple(imgpts[0].ravel().astype(int))
        cv2.line(vis, p0, tuple(imgpts[1].ravel().astype(int)), (255,0,0), 2)
        cv2.line(vis, p0, tuple(imgpts[2].ravel().astype(int)), (0,255,0), 2)
        cv2.line(vis, p0, tuple(imgpts[3].ravel().astype(int)), (0,0,255), 2)
        return T, vis
    return T, bgr

# --- Hand-eye calibration orchestrator ---------------------------------------

METHOD_MAP = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}

class HandEyeCalibrator:
    def __init__(self, robot: XArm6Controller, cam: RealSenseCamera,
                 rows: int, cols: int, square_size_m: float, method: str = 'tsai',
                 save_dir: str = './handeye_output', preview: bool = True):
        self.robot = robot
        self.cam = cam
        self.rows = rows
        self.cols = cols
        self.square = square_size_m
        self.method = METHOD_MAP.get(method.lower(), cv2.CALIB_HAND_EYE_TSAI)
        self.preview = preview
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        self.samples = []  # list of dicts: {'base_T_ee':..., 'target_T_cam':...}

    def collect_samples(self, target_count: int):
        K, dist, w, h = self.cam.intrinsics
        print("\n=== Capture ===")
        print("Move the robot through diverse wrist orientations while keeping the board fixed.")
        print("Press [Enter/Space] to capture, [v] verify, [s] save raw, [q/ESC] to finish.")
        win = 'Camera'
        if self.preview:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win, 960, 720)

        while True:
            frame = self.cam.get_color()
            if frame is None:
                print("[WARN] Failed to get camera frame.")
                continue

            T_t2c, vis = detect_chessboard_T_target2cam(frame, self.rows, self.cols, self.square, K, dist, draw=True)
            if self.preview:
                info = f"samples: {len(self.samples)}/{target_count}"
                cv2.putText(vis, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.imshow(win, vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (13, 32):  # Enter or Space -> capture
                if T_t2c is None:
                    print("[HINT] Chessboard not detected. Adjust pose and try again.")
                    continue
                base_T_ee = self.robot.get_base_T_gripper()
                self.samples.append({'base_T_ee': base_T_ee, 'target_T_cam': T_t2c})
                print(f"Captured #{len(self.samples)}")
                if len(self.samples) >= target_count:
                    print("Reached target sample count.")
                    break

            elif key in (ord('q'), 27):  # q or ESC
                if len(self.samples) < 3:
                    print("Need at least 3 samples to calibrate.")
                    continue
                break
            elif key == ord('v'):
                self.quick_verify()
            elif key == ord('s'):
                self._save_raw_samples()

        if self.preview:
            cv2.destroyWindow(win)

    def calibrate(self):
        if len(self.samples) < 3:
            raise RuntimeError("Not enough samples. Collect at least 3.")
        # OpenCV expects lists of R,t, not full matrices
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        for s in self.samples:
            T_ge = s['base_T_ee']  # base<-ee
            R_gripper2base.append(T_ge[:3, :3])
            t_gripper2base.append(T_ge[:3, 3])
            T_tc = s['target_T_cam']  # cam<-target
            R_target2cam.append(T_tc[:3, :3])
            t_target2cam.append(T_tc[:3, 3])

        # calibrateHandEye returns R_cam2gripper, t_cam2gripper
        R_c2g, t_c2g = cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
                                            R_target2cam, t_target2cam,
                                            method=self.method)
        ee_T_cam = np.eye(4, dtype=np.float64)
        ee_T_cam[:3, :3] = R_c2g
        ee_T_cam[:3, 3] = t_c2g.reshape(3)

        self._save_results(ee_T_cam)
        print("\n=== Calibration result (ee_T_cam) ===")
        print(np.array_str(ee_T_cam, precision=6, suppress_small=True))
        return ee_T_cam

    def quick_verify(self):
        if not self.samples:
            print("[Verify] No samples yet.")
            return
        K, dist, _, _ = self.cam.intrinsics
        frame = self.cam.get_color()
        if frame is None:
            print("[Verify] No frame.")
            return
        T_t2c, vis = detect_chessboard_T_target2cam(frame, self.rows, self.cols, self.square, K, dist, draw=True)
        if T_t2c is None:
            print("[Verify] Chessboard not found.")
            return

        # If we have a previous result saved, load it; else do a provisional fit
        res_path = os.path.join(self.save_dir, 'ee_T_cam.npz')
        if os.path.exists(res_path):
            data = np.load(res_path)
            ee_T_cam = data['ee_T_cam']
        else:
            print("[Verify] Running provisional calibration from current samples...")
            ee_T_cam = self.calibrate()

        base_T_ee = self.robot.get_base_T_gripper()
        base_T_cam_pred = base_T_ee @ ee_T_cam

        # We don't know base_T_target, so compare cam pose prediction vs measured cam pose w.r.t. target.
        # Compute cam_T_target from measured T_target2cam:
        cam_T_target_meas = np.linalg.inv(T_t2c)
        # For visualization, just print cam z distance and orientation difference as a sanity check.
        trans = cam_T_target_meas[:3, 3]
        print(f"[Verify] cam_T_target (meas) t= {trans} (m)")
        # Show the preview frame
        cv2.imshow('Verify', vis)
        cv2.waitKey(500)
        cv2.destroyWindow('Verify')

    def _save_raw_samples(self):
        ts = int(time.time())
        path = os.path.join(self.save_dir, f'samples_{ts}.npz')
        mats = {
            'base_T_ee': np.stack([s['base_T_ee'] for s in self.samples]),
            'target_T_cam': np.stack([s['target_T_cam'] for s in self.samples])
        }
        np.savez(path, **mats)
        print(f"Raw samples saved -> {path}")

    def _save_results(self, ee_T_cam):
        np.savez(os.path.join(self.save_dir, 'ee_T_cam.npz'), ee_T_cam=ee_T_cam, timestamp=time.time())
        # Also write an OpenCV YAML for convenience
        fs = cv2.FileStorage(os.path.join(self.save_dir, 'ee_T_cam.yaml'), cv2.FILE_STORAGE_WRITE)
        fs.write("ee_T_cam", ee_T_cam)
        fs.release()
        # JSON
        with open(os.path.join(self.save_dir, 'ee_T_cam.json'), 'w') as f:
            json.dump({'ee_T_cam': ee_T_cam.tolist()}, f, indent=2)
        print(f"Saved results to {self.save_dir} (NPZ/YAML/JSON)")


# --- Main --------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description='Eye-in-hand hand–eye calibration for xArm6 + RealSense')
    p.add_argument('--robot_ip', type=str, required=True, help='xArm6 controller IP, e.g., 192.168.1.197')
    p.add_argument('--rows', type=int, default=9, help='Chessboard rows (inner corners)')
    p.add_argument('--cols', type=int, default=6, help='Chessboard cols (inner corners)')
    p.add_argument('--square', type=float, default=0.025, help='Chessboard square size in meters')
    p.add_argument('--samples', type=int, default=12, help='Target number of capture poses')
    p.add_argument('--method', type=str, default='tsai', choices=list(METHOD_MAP.keys()))
    p.add_argument('--preview', action='store_true', help='Show camera preview with detection overlay')
    return p.parse_args()


def main():
    args = parse_args()
    robot = XArm6Controller(args.robot_ip)
    cam = RealSenseCamera()
    cam.start()

    try:
        calibrator = HandEyeCalibrator(robot, cam, args.rows, args.cols, args.square, args.method, preview=args.preview)
        calibrator.collect_samples(args.samples)
        ee_T_cam = calibrator.calibrate()

        print("\nHow to use:")
        print("1) Query robot pose base_T_ee; 2) base_T_cam = base_T_ee @ ee_T_cam.")
        print("Saved in: ./handeye_output/ee_T_cam.*")

    finally:
        cam.stop()
        robot.disconnect()


if __name__ == '__main__':
    main()
# python cali.py \
#   --robot_ip 192.168.1.197 \
#   --rows 5 --cols 8 --square 0.025 \
#   --samples 15 --method tsai --preview