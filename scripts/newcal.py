import os, sys
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "lib"))

import AR4_api_fixed_snippet as AR4_api  # from lib/

import pickle, time, sys, os, re

COM_PORT = "COM6"

CAL_SPEED = 15

JOINT_TRAVEL = {
    1: (170, 170), 2: (90, 90), 3: (89, 89),
    4: (165, 165), 5: (105, 105), 6: (155, 155),
}

# Joints that start at their max in the seated position.
# These only calibrate in ONE direction (away from the seat),
# then return to halfway.
# 
# 'direction' = which way to move from seated
#   -1 means the seated position is at the positive limit,
#       so we move in the negative direction
#   +1 means the seated position is at the negative limit,
#       so we move in the positive direction
#
# Adjust these based on which direction J2/J3 are maxed at
# when the arm is in the seated position.
SINGLE_DIR_JOINTS = {
    2: {'direction': -1},  # J2 seated at positive max, move negative
    3: {'direction': -1},  # J3 seated at positive max, move negative
}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

CAL_FILE_IN = os.path.join(REPO_ROOT, "config", "ARbot.cal")
CAL_FILE_OUT = os.path.join(REPO_ROOT, "config", "ARbot_safe.cal")

LIMIT_IDX = {1:(133,134),2:(135,136),3:(137,138),4:(139,140),5:(141,142),6:(143,144)}


def load_cal():
    cal = pickle.load(open(CAL_FILE_IN, "rb"))
    if not isinstance(cal, list) or len(cal) < 188:
        raise ValueError
    return cal


def flush(robot, t=1.0):
    time.sleep(t)
    robot.ser.reset_input_buffer()
    robot.ser.reset_output_buffer()
    time.sleep(0.2)


def send_fk(robot):
    flush(robot, 0.3)
    robot.ser.write(b"FK\n")
    time.sleep(1.2)
    robot.ser.reset_input_buffer()
    for j in range(1, 7):
        robot.calibration[f"J{j}AngCur"] = "0.0"
        robot.calibration[f"J{j}PosLim"] = "170"
        robot.calibration[f"J{j}NegLim"] = "-170"


def send_sp(robot, angles):
    sp = (f"SPA{angles[0]:.3f}B{angles[1]:.3f}C{angles[2]:.3f}"
          f"D{angles[3]:.3f}E{angles[4]:.3f}F{angles[5]:.3f}G0H0I0\n")
    robot.ser.write(sp.encode())
    time.sleep(0.8)
    robot.ser.reset_input_buffer()
    for j in range(1, 7):
        robot.calibration[f"J{j}AngCur"] = str(angles[j - 1])
        robot.calibration[f"J{j}PosLim"] = "170"
        robot.calibration[f"J{j}NegLim"] = "-170"


def parse_pos(response):
    angles = {}
    for i, letter in enumerate("ABCDEF"):
        m = re.search(rf'{letter}([-\d.]+)', response)
        if m:
            angles[i + 1] = float(m.group(1))
    return angles


def query_position(robot):
    robot.ser.reset_input_buffer()
    robot.ser.reset_output_buffer()
    time.sleep(0.3)
    robot.ser.reset_input_buffer()
    robot.ser.write(b"RP\n")
    time.sleep(0.5)
    buf = ""
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if robot.ser.in_waiting:
            buf += robot.ser.read(robot.ser.in_waiting).decode("utf-8", "ignore")
            if "\n" in buf:
                line = buf.split("\n")[0].strip()
                if line.startswith("A"):
                    break
                buf = buf.split("\n", 1)[1] if "\n" in buf else ""
        time.sleep(0.05)
    return parse_pos(buf)


def reconnect(robot):
    try:
        robot.ser.close()
    except:
        pass
    time.sleep(1.5)
    robot.ser.open()
    time.sleep(1.0)
    robot.ser.reset_input_buffer()
    robot.ser.reset_output_buffer()


def release_estop(robot):
    flush(robot, 1.5)
    input("  Release E-stop, press Enter...")
    reconnect(robot)
    robot.e_stop_active = False


def do_move(robot, targets, cur, speed=CAL_SPEED):
    send_fk(robot)
    time.sleep(0.2)
    send_sp(robot, cur)
    time.sleep(0.3)
    robot.e_stop_active = False
    for j in range(1, 7):
        robot.calibration[f"J{j}AngCur"] = str(cur[j - 1])
    try:
        robot.move_r(targets[0], targets[1], targets[2],
                     targets[3], targets[4], targets[5],
                     spd_prefix='Sp', speed=speed,
                     acceleration=10, deceleration=10, acc_ramp=50)
    except:
        robot.e_stop_active = True
    return robot.e_stop_active


def find_limit(robot, joint, direction, cur):
    pos, neg = JOINT_TRAVEL[joint]
    target = pos * direction if direction > 0 else -neg
    input(f"  J{joint} -> {target} -- Enter to start, E-STOP to mark ")
    targets = list(cur)
    targets[joint - 1] = target
    estop = do_move(robot, targets, cur)
    flush(robot, 0.5)

    if estop:
        release_estop(robot)
        actual = query_position(robot)
        stopped = actual.get(joint, None)
        if stopped is None or abs(stopped) < 0.01:
            stopped = float(input(f"  Enter J{joint} angle: ") or "0")
        new_cur = list(cur)
        new_cur[joint - 1] = stopped
        send_sp(robot, new_cur)
        print(f"  Recorded: {stopped:.1f}")
        return stopped, new_cur
    else:
        stopped = float(robot.calibration.get(f"J{joint}AngCur", target))
        new_cur = list(cur)
        for j in range(1, 7):
            new_cur[j - 1] = float(robot.calibration.get(f"J{j}AngCur", cur[j - 1]))
        print(f"  End of travel: {stopped:.1f}")
        return stopped, new_cur


def return_home(robot, cur):
    home = [0.0] * 6
    flush(robot, 1.0)
    if do_move(robot, home, cur, speed=20):
        release_estop(robot)
    send_sp(robot, home)
    for j in range(1, 7):
        robot.calibration[f"J{j}AngCur"] = "0.0"
    return home


def return_to_angle(robot, cur, joint, angle):
    """Move a single joint to a specific angle, keep others at current."""
    targets = list(cur)
    targets[joint - 1] = angle
    flush(robot, 0.5)
    if do_move(robot, targets, cur, speed=20):
        release_estop(robot)
    new_cur = list(cur)
    for j in range(1, 7):
        try:
            new_cur[j - 1] = float(robot.calibration.get(f"J{j}AngCur", targets[j - 1]))
        except:
            new_cur[j - 1] = targets[j - 1]
    send_sp(robot, new_cur)
    return new_cur


def save(base_cal, safe_pos):
    cal = list(base_cal)
    for j, (p, n) in safe_pos.items():
        pi, ni = LIMIT_IDX[j]
        if p is not None:
            cal[pi] = str(round(abs(p), 2))
        if n is not None:
            cal[ni] = str(round(abs(n), 2))
    pickle.dump(cal, open(CAL_FILE_OUT, "wb"))


def run():
    base_cal = load_cal()
    input("Place robot in SEATED position, press Enter to connect...")
    robot = AR4_api.AR4(COM_PORT)
    robot.open()
    time.sleep(1.5)
    flush(robot)
    send_fk(robot)
    for j in range(1, 7):
        robot.set_joint_open_loop(j)
    print("Ready. Home = 0 (seated position)\n")

    cur, safe_pos = [0.0] * 6, {}

    for j in range(1, 7):
        print(f"--- J{j} ---")

        if j in SINGLE_DIR_JOINTS:
            # ── SINGLE DIRECTION (J2, J3) ────────────────────────
            # These joints start at their max in the seated position.
            # Only move in one direction, then return to halfway.
            direction = SINGLE_DIR_JOINTS[j]['direction']
            dir_label = "negative" if direction < 0 else "positive"
            print(f"  Single-direction calibration ({dir_label} only)")

            a, cur = find_limit(robot, j, direction, cur)

            # The seated position (0) is one limit,
            # the recorded angle is the other limit
            if direction < 0:
                # Seated is positive limit, recorded is negative limit
                pos_limit = 0.0
                neg_limit = a
                safe_pos[j] = (pos_limit, neg_limit)
            else:
                # Seated is negative limit, recorded is positive limit
                pos_limit = a
                neg_limit = 0.0
                safe_pos[j] = (pos_limit, neg_limit)

            print(f"  J{j}: {min(pos_limit, neg_limit):.1f} to {max(pos_limit, neg_limit):.1f}")

            # Return to halfway between seated (0) and the limit we found
            halfway = a / 2.0
            print(f"  Returning J{j} to halfway: {halfway:.1f}")
            cur = return_to_angle(robot, cur, j, halfway)

        else:
            # ── NORMAL TWO-DIRECTION (J1, J4, J5, J6) ────────────
            a, cur = find_limit(robot, j, +1, cur)
            b, cur = find_limit(robot, j, -1, cur)
            safe_pos[j] = (max(a, b), min(a, b))
            print(f"  J{j}: {min(a, b):.1f} to {max(a, b):.1f}")
            # Only zero this joint, keep others (especially J2/J3) where they are
            print(f"  Returning J{j} to 0")
            cur = return_to_angle(robot, cur, j, 0.0)

        save(base_cal, safe_pos)

        if j < 6 and input("  Next? (Enter/q): ").strip().lower() == 'q':
            break

    cur = return_home(robot, cur)
    for j in range(1, 7):
        robot.set_joint_closed_loop(j)

    print("\nLimits:")
    for j, (p, n) in safe_pos.items():
        print(f"  J{j}: {n:.1f} to {p:.1f}")
    robot.close()


if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        print("\nStopped.")
