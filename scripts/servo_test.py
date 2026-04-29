"""
AR4 Servo Gripper Test
=======================
Simple test to verify the DS3225 servo works.
Opens and closes the gripper a few times.

Servo 0:  0 degrees = closed, 55 degrees = open
"""

import serial
import time

NANO_COM_PORT = "COM7"
NANO_BAUD = 115200


def servo_cmd(ser, number, position):
    """Send servo command to Arduino Nano."""
    command = f"SV{number}P{position}\r\n"
    ser.write(command.encode())
    time.sleep(0.5)
    # Read response if any
    if ser.in_waiting:
        response = ser.read(ser.in_waiting).decode('utf-8', 'ignore').strip()
        print(f"    Nano: {response}")
    ser.reset_input_buffer()


def main():
    print("=" * 40)
    print("  SERVO GRIPPER TEST")
    print("=" * 40)

    print(f"\n  Connecting to Nano on {NANO_COM_PORT}...")
    ser = serial.Serial(NANO_COM_PORT, NANO_BAUD, timeout=1)
    time.sleep(3)  # Nano resets on serial connect, wait for it
    ser.reset_input_buffer()
    print("  Connected!\n")

    print("  Commands:")
    print("    o = open (55 degrees)")
    print("    c = close (0 degrees)")
    print("    0-55 = go to specific angle")
    print("    q = quit\n")

    while True:
        cmd = input("  > ").strip().lower()

        if cmd == 'q':
            break
        elif cmd == 'o':
            print("  Opening...")
            servo_cmd(ser, 0, 55)
        elif cmd == 'c':
            print("  Closing...")
            servo_cmd(ser, 0, 0)
        else:
            try:
                angle = int(cmd)
                if 0 <= angle <= 55:
                    print(f"  Moving to {angle} degrees...")
                    servo_cmd(ser, 0, angle)
                else:
                    print("  Range: 0 (closed) to 55 (open)")
            except ValueError:
                print("  Type o, c, 0-55, or q")

    print("  Opening gripper before disconnect...")
    servo_cmd(ser, 0, 55)
    ser.close()
    print("  Done.")


if __name__ == "__main__":
    main()
