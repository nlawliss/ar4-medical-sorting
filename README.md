# AR4 Medical Equipment Sorting System

Automated sorting of medical instruments (scissors, syringes, tweezers) using an Annin Robotics AR4 robot arm, Intel RealSense D405 depth camera, YOLOv8 object detection, and a servo gripper.

## Overview

The system scans a nurse's tray with a wrist-mounted depth camera, identifies medical instruments using a trained YOLO model, maps camera coordinates to robot joint angles via linear regression, then picks and sorts each item into the correct bin. The process repeats continuously until the tray is empty.

**Pipeline:** D405 Camera → YOLOv8 Detection → Linear Regression Mapping → AR4 Joint Control → Servo Gripper → Sorted Bins

## Hardware

- **Robot arm:** Annin Robotics AR4 (6-DOF), controlled via Teensy 4.1
- **Camera:** Intel RealSense D405 (short-range depth), wrist-mounted
- **Gripper:** DS3225 servo gripper, controlled via Arduino Nano
- **Tray:** Medical instrument tray mounted on cart
- **Bins:** 3 sorting bins (scissors, syringes, tweezers)

## Setup

### Requirements

```bash
pip install -r requirements.txt
```

### Hardware connections

| Device | COM Port | Baud Rate |
|--------|----------|-----------|
| Teensy 4.1 (arm) | COM6 | 9600 |
| Arduino Nano (gripper) | COM7 | 9600 |
| D405 camera | USB | N/A |

Update COM ports in each script's configuration section if yours differ.

### File locations

| File | Purpose |
|------|---------|
| `config/ARbot.cal` | Base calibration file (from AR4 GUI software) |
| `config/ARbot_safe.cal` | Safe calibration with tested joint limits |
| `config/cam_calibration.json` | Camera-to-joint angle mapping |
| `model/PROTO1.pt` | Trained YOLOv8 model |
| `lib/AR4_api_fixed_snippet.py` | AR4 Python API |

## Usage

### Startup procedure (every power cycle)

```
1. Power on robot, place arm in seated position
2. python scripts/newcal.py                          # Calibrate joints
3. python scripts/move_teach.py                      # Verify positions
   > goto over_tray                                  # Check key positions
   > goto center_tray
   > q
4. python scripts/ar4_continuous.py                  # Run sorting
```

### Scripts

#### `scripts/newcal.py` — Joint calibration

Calibrates joint limits from the seated position using E-stop. J2/J3 use single-direction calibration since they start at their mechanical max.

```bash
python scripts/newcal.py
```

#### `scripts/move_teach.py` — Teach positions

Interactive teach pendant for jogging the arm and saving named positions. Positions persist across sessions in `ar4_saved_positions.json`.

```bash
python scripts/move_teach.py
```

Commands: `J1 J2 J3 J4 J5 J6` (move), `save <name>`, `goto <name>`, `list`, `export`, `where`, `home`, `q`

#### `scripts/ar4_scan.py` — Camera scan & calibration

Two modes:

```bash
python scripts/ar4_scan.py                    # Scan tray, save detections
python scripts/ar4_scan.py --calibrate        # Add calibration points
python scripts/ar4_scan.py --calibrate --clear # Fresh calibration
python scripts/ar4_scan.py --camera-only      # Camera preview only
```

Calibration process: place one object → scan → jog arm to touch it → `save` → repeat for 20+ points across the tray.

#### `scripts/ar4_continuous.py` — Main pipeline

Continuous scan-pick-sort loop. Scans the tray, picks the highest-confidence detection, sorts it to the correct bin, rescans until the tray is empty.

```bash
python scripts/ar4_continuous.py
```

#### `scripts/servo_test.py` — Gripper testing

Interactive servo control for finding grip values.

```bash
python scripts/servo_test.py
```

Commands: `o` (open), `c` (close), `0-180` (specific angle), `q` (quit)

#### `scripts/fix_j2_limit.py` — Adjust joint limits

Quick script to modify joint limits in the calibration file without re-running full calibration.

```bash
python scripts/fix_j2_limit.py
```

## Calibration details

### Joint calibration (`newcal.py`)

- Custom E-stop based calibration replaces standard limit switch homing
- J2/J3: single-direction calibration from seated (mechanical max) position, returns to halfway
- J1/J4/J5/J6: standard two-direction calibration
- Safe limits saved to `config/ARbot_safe.cal`

### Camera calibration (`ar4_scan.py --calibrate`)

- Maps D405 camera 3D coordinates to AR4 joint angles
- Uses linear regression: `Ji = a*cam_x + b*cam_y + c` for each joint
- Minimum 3 points required, 20+ recommended for accuracy
- Points saved to `config/cam_calibration.json`
- Calibration must be redone if camera mount position changes

### Orientation detection

- Bounding box aspect ratio determines object orientation
- Wider than tall (aspect > 1.3) triggers J6 rotation for perpendicular grip
- Applied automatically during the pick-and-place pipeline

## Key positions

| Position | Description |
|----------|-------------|
| `home` | Seated position (0,0,0,0,0,0) — power-on/off position |
| `over_tray` | Safe waypoint above tray — used between every pick/place |
| `scan_position` | Wrist tilted down for camera to view tray |
| `center_tray` | Gripper touching center of tray surface |
| `bin_left/middle/right` | Drop positions above each sorting bin |

## Architecture

```
┌─────────────┐    ┌─────────────┐    ┌──────────────────┐
│  D405 Camera │───▸│  YOLOv8     │───▸│  Linear          │
│  RGB + Depth │    │  Detection  │    │  Regression      │
└─────────────┘    └─────────────┘    │  (cam → joints)  │
                                       └────────┬─────────┘
                                                │
                   ┌─────────────┐    ┌─────────▾─────────┐
                   │  Servo      │◂───│  AR4 Robot Arm    │
                   │  Gripper    │    │  6-DOF Control    │
                   └──────┬──────┘    └───────────────────┘
                          │
              ┌───────────┼───────────┐
              ▾           ▾           ▾
         ┌─────────┐ ┌─────────┐ ┌─────────┐
         │Scissors │ │Syringes │ │Tweezers │
         │  Bin    │ │  Bin    │ │  Bin    │
         └─────────┘ └─────────┘ └─────────┘
```

## Gripper values

| Object | Grip angle | Open angle |
|--------|-----------|------------|
| Scissors | 120 | 180 |
| Syringes | 130 | 180 |
| Tweezers | 120 | 180 |

## Safety

- Live joint limit checking against `ARbot_safe.cal` on every move
- `home()` bypasses limit checks (seated position is always safe)
- Safe waypoint routing through `over_tray` between all picks and places
- `MAX_CYCLES = 20` prevents infinite loops
- E-stop detection halts motion immediately

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `serial.Serial` error | `pip uninstall serial && pip install pyserial` |
| `ARbot.cal` not found | Copy from AR4 GUI software directory |
| Servo not moving | Check Nano power switch, verify 9600 baud |
| Arm misses consistently | Add more calibration points in problem areas |
| `BLOCKED` on home | Home bypasses safety — update script if needed |
| Camera not detected | Check USB connection, try different port |
