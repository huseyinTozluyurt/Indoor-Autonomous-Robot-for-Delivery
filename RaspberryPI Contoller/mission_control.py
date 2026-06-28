import serial
import time
import re


# ============================================================
# Serial Configuration
# ============================================================

ARDUINO_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


# ============================================================
# Default Robot Settings
# ============================================================

DEFAULT_FORWARD_SPEED = 180
DEFAULT_TURN_SPEED = 240

DEFAULT_FORWARD_TIME_MS = 1000
DEFAULT_TURN_TIME_MS = 1200

READ_EXTRA_TIME_S = 0.35


# ============================================================
# IMU / Yaw Parsing
# ============================================================

def parse_yaw(line: str):
    """
    Expected Arduino line:
        PID,Yaw=-3.39,Target=0.00,Error=3.39,dt=0.020,Correction=60.00
    """

    match = re.search(r"Yaw=([-+]?\d+\.?\d*)", line)

    if not match:
        return None

    return float(match.group(1))


# ============================================================
# Serial Helper Functions
# ============================================================

def open_serial():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(2.0)
        ser.reset_input_buffer()
        print(f"Connected to Arduino on {ARDUINO_PORT}")
        return ser

    except serial.SerialException as e:
        print(f"Could not open {ARDUINO_PORT}: {e}")
        print()
        print("Try:")
        print("  sudo chmod 666 /dev/ttyUSB0")
        print()
        print("Or permanent fix:")
        print("  sudo usermod -aG dialout $USER")
        print("  sudo reboot")
        return None


def send_raw_command(ser, command: str):
    command = command.strip()

    if not command:
        return

    print(f"SEND: {command}")
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def read_arduino_for_duration(ser, duration_s: float):
    """
    Read Arduino serial output for a fixed duration.
    """

    lines = []
    start_time = time.time()

    while time.time() - start_time < duration_s:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line:
                print(f" -> [ARDUINO]: {line}")
                lines.append(line)

        time.sleep(0.005)

    return lines


def read_until_done_or_timeout(ser, timeout_s: float):
    """
    Read Arduino output until DONE appears or timeout occurs.
    """

    lines = []
    start_time = time.time()

    while time.time() - start_time < timeout_s:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line:
                print(f" -> [ARDUINO]: {line}")
                lines.append(line)

                yaw = parse_yaw(line)
                if yaw is not None:
                    print(f" -> [IMU] Heading/Yaw: {yaw:.2f}°")

                if "DONE" in line:
                    return lines

        time.sleep(0.005)

    print(" -> Timeout reached while waiting for Arduino.")
    return lines


# ============================================================
# Correct Arduino Command Format
# ============================================================

def send_drive_command(ser, cmd: str, left_speed: int, right_speed: int, duration_ms: int):
    """
    Your Arduino expects exactly 4 arguments:

        CMD left_speed right_speed duration_ms

    Valid examples:
        F 180 180 1000
        B 180 180 1000
        L 240 240 1200
        R 240 240 1200
        P 160 160 3000
        G 180 180 1000
    """

    cmd = cmd.upper()

    if cmd not in ["F", "B", "L", "R", "P", "G"]:
        print(f"Invalid command: {cmd}")
        return []

    left_speed = int(max(0, min(255, left_speed)))
    right_speed = int(max(0, min(255, right_speed)))
    duration_ms = int(max(1, duration_ms))

    command = f"{cmd} {left_speed} {right_speed} {duration_ms}"

    send_raw_command(ser, command)

    timeout_s = duration_ms / 1000.0 + READ_EXTRA_TIME_S
    return read_until_done_or_timeout(ser, timeout_s)


def stop_robot(ser):
    send_raw_command(ser, "S")
    read_arduino_for_duration(ser, 0.25)


def reset_yaw(ser):
    send_raw_command(ser, "Z")
    read_arduino_for_duration(ser, 0.40)


# ============================================================
# Movement Functions
# ============================================================

def forward(ser, speed=DEFAULT_FORWARD_SPEED, duration_ms=DEFAULT_FORWARD_TIME_MS):
    print()
    print(f"⬆️ Forward | Left: {speed} | Right: {speed} | Time: {duration_ms}ms")
    return send_drive_command(ser, "F", speed, speed, duration_ms)


def backward(ser, speed=DEFAULT_FORWARD_SPEED, duration_ms=DEFAULT_FORWARD_TIME_MS):
    print()
    print(f"⬇️ Backward | Left: {speed} | Right: {speed} | Time: {duration_ms}ms")
    return send_drive_command(ser, "B", speed, speed, duration_ms)


def turn_left(ser, speed=DEFAULT_TURN_SPEED, duration_ms=DEFAULT_TURN_TIME_MS):
    print()
    print(f"🔄 Executing Rotational Turn | Direction: L | Speed: {speed} | Time: {duration_ms}ms")
    print(f" -> Correct Arduino command will be: L {speed} {speed} {duration_ms}")
    return send_drive_command(ser, "L", speed, speed, duration_ms)


def turn_right(ser, speed=DEFAULT_TURN_SPEED, duration_ms=DEFAULT_TURN_TIME_MS):
    print()
    print(f"🔄 Executing Rotational Turn | Direction: R | Speed: {speed} | Time: {duration_ms}ms")
    print(f" -> Correct Arduino command will be: R {speed} {speed} {duration_ms}")
    return send_drive_command(ser, "R", speed, speed, duration_ms)


def pid_forward_reset(ser, speed=DEFAULT_FORWARD_SPEED, duration_ms=3000):
    """
    P command:
        PID forward WITH yaw reset.
    """

    print()
    print(f"🧭 PID Forward WITH yaw reset | Speed: {speed} | Time: {duration_ms}ms")
    return send_drive_command(ser, "P", speed, speed, duration_ms)


def pid_forward_no_reset(ser, speed=DEFAULT_FORWARD_SPEED, duration_ms=1000):
    """
    G command:
        PID forward WITHOUT yaw reset.
    """

    print()
    print(f"🧭 PID Forward WITHOUT yaw reset | Speed: {speed} | Time: {duration_ms}ms")
    return send_drive_command(ser, "G", speed, speed, duration_ms)


# ============================================================
# Manual Custom Command Helpers
# ============================================================

def custom_drive_command(ser):
    print()
    print("Custom drive command format:")
    print("  CMD left_speed right_speed duration_ms")
    print("Example:")
    print("  F 180 180 1000")
    print("  L 240 240 1200")
    print("  R 240 240 1200")
    print("  P 160 160 3000")
    print("  G 180 180 1000")
    print()

    cmd = input("CMD [F/B/L/R/P/G]: ").strip().upper()

    if cmd not in ["F", "B", "L", "R", "P", "G"]:
        print("Invalid command.")
        return

    try:
        left_speed = int(input("Left speed  [0-255]: ").strip())
        right_speed = int(input("Right speed [0-255]: ").strip())
        duration_ms = int(input("Duration ms       : ").strip())
    except ValueError:
        print("Invalid number.")
        return

    send_drive_command(ser, cmd, left_speed, right_speed, duration_ms)


def raw_command(ser):
    print()
    print("Raw command examples:")
    print("  S")
    print("  Z")
    print("  F 180 180 1000")
    print("  L 240 240 1200")
    print("  R 240 240 1200")
    print()

    command = input("raw> ").strip()

    if not command:
        return

    send_raw_command(ser, command)
    read_arduino_for_duration(ser, 1.0)


# ============================================================
# Mission Control Menu
# ============================================================

def print_menu():
    print()
    print("============== DeliveryBot Mission Control ==============")
    print("1  - Forward")
    print("2  - Backward")
    print("3  - Turn Left")
    print("4  - Turn Right")
    print("5  - PID Forward WITH yaw reset   (P)")
    print("6  - PID Forward WITHOUT yaw reset (G)")
    print("7  - Reset Yaw                    (Z)")
    print("8  - Stop                         (S)")
    print("9  - Custom drive command")
    print("10 - Raw command")
    print("q  - Quit")
    print("=========================================================")
    print()


def handle_menu_choice(ser, choice: str):
    choice = choice.strip().lower()

    if choice == "1":
        forward(ser)

    elif choice == "2":
        backward(ser)

    elif choice == "3":
        turn_left(ser)

    elif choice == "4":
        turn_right(ser)

    elif choice == "5":
        pid_forward_reset(ser, speed=160, duration_ms=3000)

    elif choice == "6":
        pid_forward_no_reset(ser, speed=160, duration_ms=3000)

    elif choice == "7":
        print()
        print("🧭 Resetting yaw reference...")
        reset_yaw(ser)

    elif choice == "8":
        print()
        print("🛑 Stopping robot...")
        stop_robot(ser)

    elif choice == "9":
        custom_drive_command(ser)

    elif choice == "10":
        raw_command(ser)

    elif choice == "q":
        stop_robot(ser)
        return False

    else:
        print("Unknown option.")

    return True


# ============================================================
# Main
# ============================================================

def main():
    ser = open_serial()

    if ser is None:
        return

    running = True

    try:
        while running:
            print_menu()
            choice = input("mission_control> ")
            running = handle_menu_choice(ser, choice)

            print()
            print("Execution finished successfully.")

    except KeyboardInterrupt:
        print("\nCTRL+C detected. Stopping robot.")
        stop_robot(ser)

    finally:
        stop_robot(ser)
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
