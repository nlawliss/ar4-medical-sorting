import os, sys
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "lib"))

"""
AR4 Continuous Pick and Place
===============================
Scans the tray, picks one object, scans again, repeats until tray is empty.
Camera, Teensy, and Nano all connected at the same time.

Flow:
  Home -> OverTray
  loop:
    scan_position -> scan
    if nothing detected: break
    OverTray -> pick (grip) -> OverTray -> bin (release) -> OverTray
  OverTray -> Home

Requirements:
    - cam_calibration.json
    - ARbot_safe.cal
    - PROTO1.pt (YOLO model)
    - D405, Teensy (COM6), Nano (COM7) all plugged in
"""

import time
import sys
import os
import json
import serial
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import AR4_api_fixed_snippet as AR4_api  # from lib/
import pickle


# ═════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═════════════════════════════════════════════════════════════════════

COM_PORT = "COM6"
NANO_COM_PORT = "COM7"
CAL_PATH = os.path.join(REPO_ROOT, "config", "ARbot_safe.cal")
CALIBRATION_FILE = os.path.join(REPO_ROOT, "config", "cam_calibration.json")

SAFETY_MARGIN = 2.0
ARM_SPEED = 25

# Camera
MODEL_PATH = os.path.join(REPO_ROOT, "model", "PROTO1.pt")
IMGSZ = 640
CONF_THRESHOLD = 0.25
IMG_WIDTH = 640
IMG_HEIGHT = 480
FPS = 30
MIN_DEPTH = 0.05
MAX_DEPTH = 2.0
ROI_CENTER_RATIO = 0.30

# Scan timing per cycle
SCAN_DURATION = 2.0    # shorter than before since we rescan each cycle
SCAN_SETTLE = 0.5

# Safety
MAX_CYCLES = 20        # Don't loop forever

LIMIT_IDX = {1:(133,134),2:(135,136),3:(137,138),4:(139,140),5:(141,142),6:(143,144)}

# Taught positions
POSITIONS = {
    'home':          [  0.000,   0.000,   0.000,   0.000,   0.000,   0.000],
    'over_tray':     [ 40.005,  25.002, -37.998,   0.000,  40.005,   0.000],
    'bin_right':     [ -9.990,  30.006, -26.001,   0.000,  25.000,   0.000],
    'bin_middle':    [  0.000,  30.006, -26.001,   0.000,  30.009,   0.000],
    'bin_left':      [ 12.004,  30.006, -26.001,   0.000,  25.000,   0.000],
    'center_tray':   [ 45.000,  46.305, -24.993,   0.000,  20.014,   0.000],
    'scan_position': [ 45.000,  30.006, -31.995,   0.000,  43.001,   0.000],
}

SCAN_POSITION = POSITIONS['scan_position']

CLASS_TO_BIN = {
    'scissor': 'bin_right',
    'syringe': 'bin_middle',
    'tweezer': 'bin_left',
}

GRIPPER_OPEN = 180
GRIPPER_GRIP = {
    'scissor': 120,
    'syringe': 130,
    'tweezer': 120,
}
GRIPPER_SETTLE = 0.8


# ═════════════════════════════════════════════════════════════════════
# GRIPPER
# ═════════════════════════════════════════════════════════════════════

class Gripper:
    def __init__(self, port=NANO_COM_PORT):
        self.port = port
        self.ser = None

    def connect(self):
        print(f"  Connecting gripper on {self.port}...")
        self.ser = serial.Serial(self.port, 9600, timeout=1)
        time.sleep(3)
        self.ser.reset_input_buffer()
        print("  Gripper connected!")

    def command(self, position):
        cmd = f"SV0P{position}\r\n"
        self.ser.write(cmd.encode())
        time.sleep(GRIPPER_SETTLE)
        if self.ser.in_waiting:
            resp = self.ser.read(self.ser.in_waiting).decode('utf-8', 'ignore').strip()
            print(f"    Nano: {resp}")
        self.ser.reset_input_buffer()

    def open(self):
        print(f"  Gripper OPEN ({GRIPPER_OPEN})")
        self.command(GRIPPER_OPEN)

    def grip(self, class_name):
        angle = GRIPPER_GRIP.get(class_name, 145)
        print(f"  Gripper CLOSE for {class_name} ({angle})")
        self.command(angle)

    def close(self):
        if self.ser:
            self.open()
            self.ser.close()
            self.ser = None


# ═════════════════════════════════════════════════════════════════════
# AR4 SAFE CONTROLLER
# ═════════════════════════════════════════════════════════════════════

class AR4Safe:
    def __init__(self, port=COM_PORT, cal_path=CAL_PATH, margin=SAFETY_MARGIN):
        self.port = port
        self.cal_path = cal_path
        self.margin = margin
        self.robot = None
        self.cur = [0.0] * 6

    def read_limits(self):
        cal = pickle.load(open(self.cal_path, "rb"))
        limits = {}
        for j, (pi, ni) in LIMIT_IDX.items():
            limits[j] = (float(cal[pi]), float(cal[ni]))
        return limits

    def check(self, angles):
        limits = self.read_limits()
        errors = []
        for j in range(1, 7):
            pos_lim, neg_lim = limits[j]
            a = angles[j - 1]
            if a > (pos_lim - self.margin):
                errors.append(f"J{j}={a:.1f} exceeds +{pos_lim - self.margin:.1f}")
            elif a < -(neg_lim - self.margin):
                errors.append(f"J{j}={a:.1f} exceeds -{neg_lim - self.margin:.1f}")
        return len(errors) == 0, errors

    def connect(self):
        self.robot = AR4_api.AR4(self.port)
        self.robot.open()
        time.sleep(1.5)
        self._flush()
        self._send_fk()
        for j in range(1, 7):
            self.robot.set_joint_open_loop(j)
        self.cur = [0.0] * 6
        print("Arm connected.")

    def close(self):
        if self.robot:
            for j in range(1, 7):
                self.robot.set_joint_closed_loop(j)
            self.robot.close()
            self.robot = None

    def move(self, angles, speed=ARM_SPEED, label=""):
        ok, errors = self.check(angles)
        if not ok:
            print(f"  BLOCKED {label}:")
            for e in errors:
                print(f"    {e}")
            return False
        if label:
            print(f"  -> {label}: {[round(a,1) for a in angles]}")
        self._send_fk()
        time.sleep(0.2)
        self._send_sp(self.cur)
        time.sleep(0.3)
        self.robot.e_stop_active = False
        for j in range(1, 7):
            self.robot.calibration[f"J{j}AngCur"] = str(self.cur[j - 1])
        try:
            self.robot.move_r(*angles, spd_prefix='Sp', speed=speed,
                              acceleration=20, deceleration=20, acc_ramp=100)
        except:
            self.robot.e_stop_active = True
        if self.robot.e_stop_active:
            print("  E-STOP!")
            return False
        for j in range(1, 7):
            try:
                self.cur[j - 1] = float(self.robot.calibration.get(f"J{j}AngCur", angles[j - 1]))
            except:
                self.cur[j - 1] = angles[j - 1]
        return True

    def goto(self, name, speed=ARM_SPEED):
        return self.move(POSITIONS[name], speed=speed, label=name)

    def home(self, speed=ARM_SPEED):
        print(f"  -> HOME: [0, 0, 0, 0, 0, 0]")
        self._send_fk()
        time.sleep(0.2)
        self._send_sp(self.cur)
        time.sleep(0.3)
        self.robot.e_stop_active = False
        for j in range(1, 7):
            self.robot.calibration[f"J{j}AngCur"] = str(self.cur[j - 1])
        try:
            self.robot.move_r(0, 0, 0, 0, 0, 0, spd_prefix='Sp', speed=speed,
                              acceleration=20, deceleration=20, acc_ramp=100)
        except:
            self.robot.e_stop_active = True
        if self.robot.e_stop_active:
            print("  E-STOP!")
            return False
        for j in range(1, 7):
            try:
                self.cur[j - 1] = float(self.robot.calibration.get(f"J{j}AngCur", 0.0))
            except:
                self.cur[j - 1] = 0.0
        return True

    def _flush(self, t=0.5):
        time.sleep(t)
        self.robot.ser.reset_input_buffer()
        self.robot.ser.reset_output_buffer()

    def _send_fk(self):
        self._flush(0.3)
        self.robot.ser.write(b"FK\n")
        time.sleep(1.2)
        self.robot.ser.reset_input_buffer()
        for j in range(1, 7):
            self.robot.calibration[f"J{j}AngCur"] = "0.0"
            self.robot.calibration[f"J{j}PosLim"] = "170"
            self.robot.calibration[f"J{j}NegLim"] = "-170"

    def _send_sp(self, angles):
        sp = (f"SPA{angles[0]:.3f}B{angles[1]:.3f}C{angles[2]:.3f}"
              f"D{angles[3]:.3f}E{angles[4]:.3f}F{angles[5]:.3f}G0H0I0\n")
        self.robot.ser.write(sp.encode())
        time.sleep(0.8)
        self.robot.ser.reset_input_buffer()
        for j in range(1, 7):
            self.robot.calibration[f"J{j}AngCur"] = str(angles[j - 1])
            self.robot.calibration[f"J{j}PosLim"] = "170"
            self.robot.calibration[f"J{j}NegLim"] = "-170"


# ═════════════════════════════════════════════════════════════════════
# CAMERA
# ═════════════════════════════════════════════════════════════════════

class TrayScanner:
    def __init__(self):
        self.model = None
        self.pipeline = None
        self.align = None
        self.depth_scale = None
        self.depth_intrinsics = None

    def initialize(self):
        print(f"Loading YOLO model: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        print(f"  Classes: {self.model.names}")
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, IMG_WIDTH, IMG_HEIGHT, rs.format.z16, FPS)
        cfg.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
        profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("  Camera ready.")

    def scan(self, duration=SCAN_DURATION, show_preview=True):
        for _ in range(int(FPS * SCAN_SETTLE)):
            self.pipeline.wait_for_frames()

        all_detections = []
        start = time.time()
        while time.time() - start < duration:
            frames = self.pipeline.wait_for_frames()
            frames = self.align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            if self.depth_intrinsics is None:
                self.depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            depth_m = depth_image * self.depth_scale

            results = self.model(color_image, imgsz=IMGSZ, conf=CONF_THRESHOLD, verbose=False)
            boxes = results[0].boxes
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)

            frame_dets = []
            annotated = color_image.copy()
            for i, bbox in enumerate(xyxy):
                x1, y1, x2, y2 = map(int, bbox)
                cls_id = int(classes[i])
                confidence = float(confs[i])
                class_name = self.model.names[cls_id]
                distance = self._roi_depth(depth_m, bbox)
                if distance is None:
                    continue
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                point_3d = self._deproject(cx, cy, distance)
                if point_3d is None:
                    continue
                bbox_w = x2 - x1
                bbox_h = y2 - y1
                aspect = bbox_w / max(bbox_h, 1)
                needs_rotate = aspect > 1.3
                frame_dets.append({
                    'class': class_name, 'class_id': cls_id,
                    'confidence': confidence,
                    'cam_x': point_3d[0], 'cam_y': point_3d[1], 'cam_z': point_3d[2],
                    'aspect': round(aspect, 2),
                    'needs_rotate': needs_rotate,
                })
                color = (0, 255, 255) if needs_rotate else (0, 255, 0)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                orient = "ROT" if needs_rotate else "OK"
                label = f"{class_name} {confidence:.2f} [{orient}]"
                cv2.putText(annotated, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            all_detections.append(frame_dets)
            if show_preview:
                cv2.imshow("Tray Scan", annotated)
                cv2.waitKey(1)

        if show_preview:
            cv2.destroyAllWindows()
        return self._average_detections(all_detections)

    def _average_detections(self, all_frames):
        from collections import defaultdict
        class_points = defaultdict(list)
        for frame_dets in all_frames:
            for det in frame_dets:
                class_points[det['class']].append(det)
        averaged = []
        for class_name, points in class_points.items():
            if len(points) < 3:
                continue
            avg_x = np.median([p['cam_x'] for p in points])
            avg_y = np.median([p['cam_y'] for p in points])
            avg_z = np.median([p['cam_z'] for p in points])
            avg_conf = np.mean([p['confidence'] for p in points])
            avg_aspect = np.median([p['aspect'] for p in points])
            rotate_pct = sum(1 for p in points if p['needs_rotate']) / len(points)
            averaged.append({
                'class': class_name,
                'cam_x': float(avg_x), 'cam_y': float(avg_y), 'cam_z': float(avg_z),
                'confidence': float(avg_conf),
                'aspect': float(avg_aspect),
                'needs_rotate': rotate_pct > 0.5,
                'n_frames': len(points),
            })
        return averaged

    def _roi_depth(self, depth_m, bbox):
        x1, y1, x2, y2 = map(int, bbox)
        h, w = y2 - y1, x2 - x1
        if h <= 0 or w <= 0:
            return None
        r = ROI_CENTER_RATIO
        cx1 = np.clip(int(x1 + (0.5 - r/2) * w), 0, depth_m.shape[1] - 1)
        cx2 = np.clip(int(x1 + (0.5 + r/2) * w), 0, depth_m.shape[1] - 1)
        cy1 = np.clip(int(y1 + (0.5 - r/2) * h), 0, depth_m.shape[0] - 1)
        cy2 = np.clip(int(y1 + (0.5 + r/2) * h), 0, depth_m.shape[0] - 1)
        roi = depth_m[cy1:cy2+1, cx1:cx2+1]
        if roi.size == 0:
            return None
        valid = roi[(roi > MIN_DEPTH) & (roi < MAX_DEPTH)]
        return float(np.median(valid)) if valid.size > 0 else None

    def _deproject(self, px, py, depth):
        if depth is None or self.depth_intrinsics is None:
            return None
        try:
            return rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [px, py], depth)
        except:
            return None

    def cleanup(self):
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()


# ═════════════════════════════════════════════════════════════════════
# CAMERA-TO-JOINT MAPPING
# ═════════════════════════════════════════════════════════════════════

def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        return None
    with open(CALIBRATION_FILE) as f:
        return json.load(f)


def camera_to_joints(cam_x, cam_y, cam_z, calibration):
    """Linear regression: Ji = a*cam_x + b*cam_y + c"""
    points = calibration['points']
    tc = POSITIONS['center_tray']
    if len(points) < 3:
        return list(tc)
    cam_pts = np.array([[p['cam_x'], p['cam_y']] for p in points])
    joint_pts = np.array([p['joints'] for p in points])
    A = np.column_stack([cam_pts, np.ones(len(cam_pts))])
    query = np.array([cam_x, cam_y, 1.0])
    result = np.zeros(6)
    for j in range(6):
        vals = joint_pts[:, j]
        if vals.max() - vals.min() < 0.5:
            result[j] = vals[0]
        else:
            coeffs, _, _, _ = np.linalg.lstsq(A, vals, rcond=None)
            result[j] = query @ coeffs
    return result.tolist()


def det_to_pick(det, calibration):
    """Convert one detection to pick angles."""
    class_name = det['class']
    joint_angles = camera_to_joints(det['cam_x'], det['cam_y'], det['cam_z'], calibration)

    # Rotate J6 if object is parallel to gripper
    if det.get('needs_rotate', False):
        joint_angles[5] = joint_angles[5] + 45.0

    return joint_angles


# ═════════════════════════════════════════════════════════════════════
# MAIN LOOP
# ═════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  AR4 CONTINUOUS PICK AND PLACE")
    print("=" * 60)

    # Load calibration
    calibration = load_calibration()
    if not calibration or len(calibration.get('points', [])) < 3:
        print(f"\n  ERROR: Need cam_calibration.json with 3+ points.")
        return
    print(f"\n  Calibration: {len(calibration['points'])} points")

    # Connect everything
    input("\n  Place robot at home, press Enter to connect...")

    arm = AR4Safe()
    arm.connect()

    gripper = Gripper()
    gripper.connect()

    scanner = TrayScanner()
    scanner.initialize()

    try:
        # Start: Home -> OverTray, gripper open
        arm.home()
        gripper.open()
        arm.goto('over_tray')

        cycle = 0
        picked_counts = {}

        while cycle < MAX_CYCLES:
            cycle += 1
            print(f"\n{'=' * 60}")
            print(f"  CYCLE {cycle}")
            print(f"{'=' * 60}")

            # Move to scan position
            arm.goto('scan_position')
            time.sleep(0.5)

            # Scan
            print(f"\n  Scanning ({SCAN_DURATION:.0f}s)...")
            detections = scanner.scan(duration=SCAN_DURATION, show_preview=True)

            # Filter to classes we know how to sort
            valid = [d for d in detections if d['class'] in CLASS_TO_BIN]

            if not valid:
                print("\n  Tray is empty. Done!")
                break

            # Pick the highest-confidence detection
            det = max(valid, key=lambda d: d['confidence'])
            class_name = det['class']
            bin_name = CLASS_TO_BIN[class_name]

            print(f"\n  Picking: {class_name} (conf={det['confidence']:.2f}) -> {bin_name}")
            if det.get('needs_rotate'):
                print(f"    [J6 rotation +45]")

            # Convert to joint angles
            pick_angles = det_to_pick(det, calibration)
            print(f"    Target joints: {[round(a,1) for a in pick_angles]}")

            # Execute
            arm.goto('over_tray')
            arm.move(pick_angles, label=f"pick {class_name}")
            gripper.grip(class_name)
            arm.goto('over_tray')
            arm.goto(bin_name)
            gripper.open()
            arm.goto('over_tray')

            picked_counts[class_name] = picked_counts.get(class_name, 0) + 1
            print(f"\n  {class_name} sorted to {bin_name}.")

        else:
            print(f"\n  Max cycles ({MAX_CYCLES}) reached.")

        # Summary
        print(f"\n{'=' * 60}")
        print(f"  SUMMARY: {sum(picked_counts.values())} objects sorted")
        for class_name, count in picked_counts.items():
            print(f"    {class_name}: {count}")
        print(f"{'=' * 60}")

    finally:
        print("\n  Returning home...")
        arm.home()
        gripper.close()
        scanner.cleanup()
        arm.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
