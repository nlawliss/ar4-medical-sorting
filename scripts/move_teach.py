import os, sys
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "lib"))

"""
move_teach.py — AR4 teach pendant with position saving.

Jog the arm to each key position and save it by name.
When done, export all positions to a Python file ready
for the pick-and-place script.

Commands:
    J1 J2 J3 J4 J5 J6  — move to joint angles
    home                — return to zero
    where               — print current position
    save <name>         — save current position with a name
    list                — show all saved positions
    delete <name>       — delete a saved position
    export              — write positions to ar4_positions.py
    q                   — return home and quit

Example session:
    > 10 20 -5 0 15 0
    > save overtray
    > 30 40 -10 0 20 0
    > save tray_center
    > save bin_left
    > export
    > q
"""

import AR4_api_fixed_snippet as AR4_api  # from lib/
import pickle, time, os, json

COM_PORT = "COM6"
CAL_PATH = os.path.join(REPO_ROOT, "config", "ARbot_safe.cal")
SAFETY_MARGIN = 2.0
POSITIONS_FILE = os.path.join(REPO_ROOT, "config", "ar4_saved_positions.json")

LIMIT_IDX = {1:(133,134),2:(135,136),3:(137,138),4:(139,140),5:(141,142),6:(143,144)}


class AR4Safe:
    """AR4 wrapper that checks every move against ARbot_safe.cal live."""

    def __init__(self, port=COM_PORT, cal_path=CAL_PATH, margin=SAFETY_MARGIN):
        self.port = port
        self.cal_path = cal_path
        self.margin = margin
        self.robot = None
        self.cur = [0.0] * 6
        self.saved = {}  # name -> [j1, j2, j3, j4, j5, j6]

        # Load previously saved positions if they exist
        if os.path.exists(POSITIONS_FILE):
            try:
                self.saved = json.load(open(POSITIONS_FILE))
                print(f"Loaded {len(self.saved)} saved positions from {POSITIONS_FILE}")
            except:
                pass

    def read_limits(self):
        cal = pickle.load(open(self.cal_path, "rb"))
        limits = {}
        for j, (pi, ni) in LIMIT_IDX.items():
            limits[j] = (float(cal[pi]), float(cal[ni]))
        return limits

    def check(self, j1, j2, j3, j4, j5, j6):
        limits = self.read_limits()
        angles = [j1, j2, j3, j4, j5, j6]
        errors = []
        for j in range(1, 7):
            pos_lim, neg_lim = limits[j]
            a = angles[j - 1]
            max_pos = pos_lim - self.margin
            max_neg = -(neg_lim - self.margin)
            if a > max_pos:
                errors.append(f"J{j}={a:.1f} exceeds +{max_pos:.1f} limit")
            elif a < max_neg:
                errors.append(f"J{j}={a:.1f} exceeds {max_neg:.1f} limit")
        return len(errors) == 0, errors

    def connect(self):
        if not os.path.exists(self.cal_path):
            raise FileNotFoundError(f"No cal file at {self.cal_path}")

        self.robot = AR4_api.AR4(self.port)
        self.robot.open()
        time.sleep(1.5)
        self._flush()
        self._send_fk()
        for j in range(1, 7):
            self.robot.set_joint_open_loop(j)
        self.cur = [0.0] * 6

        limits = self.read_limits()
        print("Connected. Limits:")
        for j, (p, n) in limits.items():
            print(f"  J{j}: -{n:.1f} to +{p:.1f}")
        print()

    def close(self):
        if self.robot:
            for j in range(1, 7):
                self.robot.set_joint_closed_loop(j)
            self.robot.close()
            self.robot = None

    def move(self, j1, j2, j3, j4, j5, j6, speed=25):
        targets = [j1, j2, j3, j4, j5, j6]
        ok, errors = self.check(j1, j2, j3, j4, j5, j6)
        if not ok:
            print("  BLOCKED:")
            for e in errors:
                print(f"    {e}")
            return False

        self._send_fk()
        time.sleep(0.2)
        self._send_sp(self.cur)
        time.sleep(0.3)
        self.robot.e_stop_active = False
        for j in range(1, 7):
            self.robot.calibration[f"J{j}AngCur"] = str(self.cur[j - 1])

        try:
            self.robot.move_r(j1, j2, j3, j4, j5, j6,
                              spd_prefix='Sp', speed=speed,
                              acceleration=20, deceleration=20, acc_ramp=100)
        except:
            self.robot.e_stop_active = True

        if self.robot.e_stop_active:
            print("  E-STOP triggered!")
            return False

        for j in range(1, 7):
            try:
                self.cur[j - 1] = float(self.robot.calibration.get(f"J{j}AngCur", targets[j - 1]))
            except:
                self.cur[j - 1] = targets[j - 1]
        return True

    def home(self, speed=20):
        """Return to home (0,0,0,0,0,0). Bypasses safety check since home is always safe."""
        targets = [0, 0, 0, 0, 0, 0]
        if self.cur[0] == 0 and self.cur[1] == 0 and self.cur[2] == 0:
            return True  # already home

        self._send_fk()
        time.sleep(0.2)
        self._send_sp(self.cur)
        time.sleep(0.3)
        self.robot.e_stop_active = False
        for j in range(1, 7):
            self.robot.calibration[f"J{j}AngCur"] = str(self.cur[j - 1])

        try:
            self.robot.move_r(0, 0, 0, 0, 0, 0,
                              spd_prefix='Sp', speed=speed,
                              acceleration=20, deceleration=20, acc_ramp=100)
        except:
            self.robot.e_stop_active = True

        if self.robot.e_stop_active:
            print("  E-STOP triggered!")
            return False

        for j in range(1, 7):
            try:
                self.cur[j - 1] = float(self.robot.calibration.get(f"J{j}AngCur", 0.0))
            except:
                self.cur[j - 1] = 0.0
        return True

    def where(self):
        print(f"  Position: J1={self.cur[0]:.2f} J2={self.cur[1]:.2f} J3={self.cur[2]:.2f} "
              f"J4={self.cur[3]:.2f} J5={self.cur[4]:.2f} J6={self.cur[5]:.2f}")
        return list(self.cur)

    def goto(self, name, speed=25):
        """Move to a previously saved position by name."""
        if name not in self.saved:
            print(f"  Unknown position '{name}'. Type 'list' to see saved positions.")
            return False
        angles = self.saved[name]
        print(f"  Going to '{name}': {[round(a, 1) for a in angles]}")
        return self.move(*angles, speed=speed)

    # ── save/load positions ──────────────────────────────────────────

    def save_position(self, name):
        self.saved[name] = [round(a, 3) for a in self.cur]
        self._persist()
        print(f"  Saved '{name}': {[round(a, 1) for a in self.cur]}")

    def delete_position(self, name):
        if name in self.saved:
            del self.saved[name]
            self._persist()
            print(f"  Deleted '{name}'")
        else:
            print(f"  '{name}' not found")

    def list_positions(self):
        if not self.saved:
            print("  No saved positions.")
            return
        print("  Saved positions:")
        for name, angles in self.saved.items():
            print(f"    {name:20s}  {[round(a, 1) for a in angles]}")

    def export_positions(self):
        """Write saved positions to a Python file for use in pick-and-place."""
        filename = os.path.join(REPO_ROOT, "config", "ar4_positions.py")
        lines = [
            '"""',
            'AR4 Saved Positions (joint angles)',
            'Generated by move_teach.py',
            f'Positions: {len(self.saved)}',
            '"""',
            '',
            '# Each position is [J1, J2, J3, J4, J5, J6] in degrees',
            '',
            'POSITIONS = {',
        ]
        for name, angles in self.saved.items():
            angles_str = ', '.join(f'{a:8.3f}' for a in angles)
            lines.append(f"    '{name}': [{angles_str}],")
        lines.append('}')
        lines.append('')

        # Generate a ready-to-use pick and place function
        lines.extend([
            '',
            '# Quick reference:',
        ])
        for name, angles in self.saved.items():
            lines.append(f'#   {name:20s}  J1={angles[0]:7.2f} J2={angles[1]:7.2f} J3={angles[2]:7.2f} '
                         f'J4={angles[3]:7.2f} J5={angles[4]:7.2f} J6={angles[5]:7.2f}')

        with open(filename, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        print(f"  Exported {len(self.saved)} positions to {filename}")
        print(f"  Use in your script:  from ar4_positions import POSITIONS")

    def _persist(self):
        """Save positions to JSON for persistence across sessions."""
        json.dump(self.saved, open(POSITIONS_FILE, 'w'), indent=2)

    # ── internals ────────────────────────────────────────────────────

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


# ── interactive mode ─────────────────────────────────────────────────

def main():
    bot = AR4Safe()
    input("Place robot at home, press Enter to connect...")
    bot.connect()

    print("Commands:")
    print("  J1 J2 J3 J4 J5 J6  — move to angles")
    print("  goto <name>         — move to saved position")
    print("  home                — return to zero")
    print("  where               — print current angles")
    print("  save <name>         — save current position")
    print("  list                — show saved positions")
    print("  delete <name>       — delete a saved position")
    print("  export              — write ar4_positions.py")
    print("  q                   — return home and quit\n")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd or cmd.lower() == 'q':
            break
        elif cmd.lower() == 'home':
            bot.home()
            print("  Home.")
        elif cmd.lower() == 'where':
            bot.where()
        elif cmd.lower() == 'list':
            bot.list_positions()
        elif cmd.lower() == 'export':
            bot.export_positions()
        elif cmd.lower().startswith('save '):
            name = cmd[5:].strip().replace(' ', '_')
            if name:
                bot.save_position(name)
            else:
                print("  Usage: save <name>")
        elif cmd.lower().startswith('delete '):
            name = cmd[7:].strip()
            if name:
                bot.delete_position(name)
            else:
                print("  Usage: delete <name>")
        elif cmd.lower().startswith('goto '):
            name = cmd[5:].strip()
            if name:
                bot.goto(name)
            else:
                print("  Usage: goto <name>")
        else:
            try:
                parts = cmd.replace(",", " ").split()
                angles = [float(x) for x in parts]
                if len(angles) != 6:
                    print("  Need 6 angles: J1 J2 J3 J4 J5 J6")
                    continue
                if bot.move(*angles):
                    print(f"  Done. At {[round(c,1) for c in bot.cur]}")
            except ValueError:
                print("  Unknown command. Type 'q' to quit.")

    print("Returning home...")
    bot.home()
    bot.close()
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")