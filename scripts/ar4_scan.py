import os, sys
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "lib"))

"""
AR4 Scan & Calibrate — Wrist-Mounted D405
===========================================
Two modes:

  1. SCAN MODE (default):
     - Arm moves to scan position above tray
     - D405 + YOLO runs for a few seconds
     - Detections are averaged and saved to scan_results.json
     - Camera can be unplugged
     - Results feed into pick-and-place

  2. CALIBRATE MODE (--calibrate):
     - Place a single object on tray
     - Scan its camera coordinates
     - Jog arm down to touch it (record joint angles)
     - Repeat for 6+ spots across tray
     - Builds camera-to-joint mapping saved to cam_calibration.json

  Options:
     --clear       Clear old calibration points before starting
     --camera-only Run camera without arm connection

Usage:
    python ar4_scan.py                        # scan mode
    python ar4_scan.py --calibrate            # calibration mode
    python ar4_scan.py --calibrate --clear    # fresh calibration
"""

import time
import sys
import os
import json
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

try:
    import AR4_api_fixed_snippet as AR4_api  # from lib/
    import pickle
    HAS_ARM = True
except ImportError:
    HAS_ARM = False
    print("WARNING: AR4_api not found. Running camera-only mode.")


# ═════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═════════════════════════════════════════════════════════════════════

MODEL_PATH = os.path.join(REPO_ROOT, "model", "PROTO1.pt")
IMGSZ = 640
CONF_THRESHOLD = 0.25
IMG_WIDTH = 640
IMG_HEIGHT = 480
FPS = 30
MIN_DEPTH = 0.05
MAX_DEPTH = 2.0
ROI_CENTER_RATIO = 0.30

COM_PORT = "COM6"
CAL_PATH = os.path.join(REPO_ROOT, "config", "ARbot_safe.cal")
SAFETY_MARGIN = 2.0
ARM_SPEED = 25

SCAN_DURATION = 5.0
SCAN_SETTLE = 1.0

SCAN_RESULTS_FILE = os.path.join(REPO_ROOT, "config", "scan_results.json")
CALIBRATION_FILE = os.path.join(REPO_ROOT, "config", "cam_calibration.json")

LIMIT_IDX = {1:(133,134),2:(135,136),3:(137,138),4:(139,140),5:(141,142),6:(143,144)}

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

    def where(self):
        return list(self.cur)

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
# CAMERA / YOLO SCANNER
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
        print(f"  Depth scale: {self.depth_scale:.6f}")
        print("  Camera ready.")

    def scan(self, duration=SCAN_DURATION, show_preview=True):
        print(f"\n  Scanning for {duration:.0f} seconds...")
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
                needs_rotate = aspect > 1.3  # wider than tall = parallel to gripper
                frame_dets.append({
                    'class': class_name, 'class_id': cls_id,
                    'confidence': confidence,
                    'cam_x': point_3d[0], 'cam_y': point_3d[1], 'cam_z': point_3d[2],
                    'pixel_x': cx, 'pixel_y': cy,
                    'bbox_w': bbox_w, 'bbox_h': bbox_h,
                    'aspect': round(aspect, 2),
                    'needs_rotate': needs_rotate,
                })
                # Color: green = no rotate, yellow = rotate
                color = (0, 255, 255) if needs_rotate else (0, 255, 0)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                orient_label = "ROT" if needs_rotate else "OK"
                label = f"{class_name} {confidence:.2f} [{bbox_w}x{bbox_h} {orient_label}]"
                cv2.putText(annotated, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            all_detections.append(frame_dets)
            if show_preview:
                elapsed = time.time() - start
                cv2.putText(annotated, f"Scanning: {elapsed:.1f}/{duration:.0f}s",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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
            needs_rotate = rotate_pct > 0.5  # majority of frames say rotate

            averaged.append({
                'class': class_name,
                'cam_x': float(avg_x), 'cam_y': float(avg_y), 'cam_z': float(avg_z),
                'confidence': float(avg_conf), 'n_frames': len(points),
                'aspect': float(avg_aspect),
                'needs_rotate': needs_rotate,
            })
            orient_str = "ROTATE J6" if needs_rotate else "OK"
            print(f"    {class_name}: ({avg_x:.4f}, {avg_y:.4f}, {avg_z:.4f})m "
                  f"conf={avg_conf:.2f} aspect={avg_aspect:.2f} [{orient_str}] "
                  f"seen in {len(points)} frames")
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
# CALIBRATION
# ═════════════════════════════════════════════════════════════════════

def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        return None
    with open(CALIBRATION_FILE) as f:
        return json.load(f)


OVERSHOOT_FACTOR = 1.10


def camera_to_joints(cam_x, cam_y, cam_z, calibration):
    """Convert camera 3D point to joint angles using linear regression.
    Then apply overshoot correction to push predictions away from center."""
    points = calibration['points']
    tc = POSITIONS['center_tray']
    if len(points) < 3:
        print("  WARNING: <3 calibration points, using center_tray.")
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
            predicted = query @ coeffs
            center = vals.mean()
            result[j] = center + (predicted - center) * OVERSHOOT_FACTOR
    return result.tolist()


def run_calibration(arm, scanner):
    print("=" * 60)
    print("  CAMERA-TO-JOINT CALIBRATION")
    print("=" * 60)
    print()
    print("  Process for each point:")
    print("    1. Place ONE object on the tray")
    print("    2. Arm scans from scan position (camera coords captured)")
    print("    3. Jog the arm to touch the object")
    print("    4. Type 'save' to record the pair")
    print("    5. Repeat for 6+ spots across the tray")
    print()

    if '--clear' in sys.argv:
        points = []
        print("  Starting fresh (old calibration cleared).")
    else:
        cal_data = load_calibration()
        if cal_data and cal_data.get('points'):
            points = cal_data['points']
            print(f"  Loaded {len(points)} existing points. (Use --clear to start fresh)")
        else:
            points = []

    while True:
        print(f"\n  --- Calibration point {len(points) + 1} ---")
        cmd = input("  Place object on tray, Enter to scan (d=done, q=quit): ").strip().lower()
        if cmd in ('q', 'd'):
            break

        arm.home()
        arm.move(SCAN_POSITION, label="scan_position")
        time.sleep(1)

        detections = scanner.scan(duration=3.0, show_preview=True)
        if not detections:
            print("  No objects detected. Try again.")
            continue

        det = max(detections, key=lambda d: d['confidence'])
        print(f"  Detected: {det['class']} at cam ({det['cam_x']:.4f}, {det['cam_y']:.4f}, {det['cam_z']:.4f})")
        print("  (Camera coords saved — arm can move now)")

        arm.goto('over_tray')
        print("\n  Jog the arm to TOUCH the object.")
        print("  Enter: J1 J2 J3 J4 J5 J6 | 'where' | 'save' | 'skip'\n")

        while True:
            jog_cmd = input("  jog> ").strip()
            if jog_cmd.lower() == 'save':
                joint_angles = arm.where()
                points.append({
                    'cam_x': det['cam_x'], 'cam_y': det['cam_y'], 'cam_z': det['cam_z'],
                    'class': det['class'],
                    'joints': [round(a, 3) for a in joint_angles],
                })
                print(f"  Saved: cam ({det['cam_x']:.4f}, {det['cam_y']:.4f}) -> joints {[round(a,1) for a in joint_angles]}")
                cal_out = {'points': points, 'scan_position': SCAN_POSITION}
                json.dump(cal_out, open(CALIBRATION_FILE, 'w'), indent=2)
                print(f"  Calibration saved ({len(points)} points)")
                break
            elif jog_cmd.lower() == 'skip':
                print("  Skipped.")
                break
            elif jog_cmd.lower() == 'where':
                pos = arm.where()
                print(f"  At: {[round(a,1) for a in pos]}")
            else:
                try:
                    angles = [float(x) for x in jog_cmd.replace(",", " ").split()]
                    if len(angles) != 6:
                        print("  Need 6 angles")
                        continue
                    arm.move(angles, label="jog")
                except ValueError:
                    print("  Invalid input")

    cal_out = {'points': points, 'scan_position': SCAN_POSITION}
    json.dump(cal_out, open(CALIBRATION_FILE, 'w'), indent=2)
    print(f"\n  Calibration complete: {len(points)} points saved to {CALIBRATION_FILE}")
    arm.home()


# ═════════════════════════════════════════════════════════════════════
# SCAN MODE
# ═════════════════════════════════════════════════════════════════════

def run_scan(arm, scanner):
    print("=" * 60)
    print("  TRAY SCAN")
    print("=" * 60)
    if arm:
        input("\n  Press Enter to move arm to scan position...")
        arm.home()
        arm.move(SCAN_POSITION, label="scan_position")
        time.sleep(1)

    detections = scanner.scan(duration=SCAN_DURATION, show_preview=True)
    if not detections:
        print("\n  No objects detected!")
        return []

    print(f"\n  Detected {len(detections)} objects:")
    for det in detections:
        orient = "ROTATE" if det.get('needs_rotate') else "OK"
        print(f"    {det['class']:10s} cam=({det['cam_x']:.4f}, {det['cam_y']:.4f}, {det['cam_z']:.4f}) "
              f"aspect={det.get('aspect', 0):.2f} [{orient}] "
              f"conf={det['confidence']:.2f} frames={det['n_frames']}")

    with open(SCAN_RESULTS_FILE, 'w') as f:
        json.dump(detections, f, indent=2)
    print(f"\n  Saved to {SCAN_RESULTS_FILE}")
    print("  Camera can be unplugged now.\n")

    cal = load_calibration()
    if cal and cal.get('points') and len(cal['points']) >= 3:
        print("  Predicted pick positions:")
        for det in detections:
            joints = camera_to_joints(det['cam_x'], det['cam_y'], det['cam_z'], cal)
            print(f"    {det['class']:10s} -> joints {[round(j,1) for j in joints]}")
    else:
        print("  No calibration. Run: python ar4_scan.py --calibrate --clear")

    if arm:
        arm.home()
    return detections


# ═════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════

def main():
    calibrate_mode = '--calibrate' in sys.argv
    camera_only = '--camera-only' in sys.argv

    scanner = TrayScanner()
    scanner.initialize()

    arm = None
    if HAS_ARM and not camera_only:
        input("Place robot at home, press Enter to connect...")
        arm = AR4Safe()
        arm.connect()

    try:
        if calibrate_mode:
            if not arm:
                print("ERROR: Calibration requires arm connection.")
                return
            run_calibration(arm, scanner)
        else:
            run_scan(arm, scanner)
    finally:
        scanner.cleanup()
        if arm:
            arm.home()
            arm.close()


if __name__ == "__main__":
    main()
