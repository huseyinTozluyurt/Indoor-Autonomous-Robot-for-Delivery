import serial
import time
import re


# =========================
# Serial Settings
# =========================

ARDUINO_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


# =========================
# Default Movement Settings
# =========================

DEFAULT_FORWARD_SPEED = 105
DEFAULT_BACKWARD_SPEED = 100
DEFAULT_TURN_SPEED = 90

DEFAULT_FORWARD_MS = 500
DEFAULT_BACKWARD_MS = 500
DEFAULT_TURN_MS = 300


# =========================
# Supervised Forward Settings
# =========================

FORWARD_STEP_MS = 300

YAW_WARNING_DEG = 4.0
YAW_CRITICAL_DEG = 7.0

CORRECTION_SPEED = 90
SMALL_CORRECTION_MS = 150
BIG_CORRECTION_MS = 250

# IMPORTANT:
# If correction goes in the wrong direction, swap these two.
POSITIVE_YAW_CORRECTION_CMD = "R"
NEGATIVE_YAW_CORRECTION_CMD = "L"


# =========================
# Serial Helpers
# =========================

def send_command(ser, command: str):
    print(f"SEND: {command}")
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def read_available_lines(ser, duration_s=0.2):
    """
    Read Arduino serial output for a short duration.
    Returns all received lines.
    """
    lines = []
    start = time.time()

    while time.time() - start < duration_s:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line:
                print("ARDUINO:", line)
                lines.append(line)

        time.sleep(0.005)

    return lines


def parse_yaw(line: str):
    """
    Expected Arduino line:
    PID,Yaw=4.75,Target=0.00,Error=-4.75,dt=0.020,Correction=-7.20
    """
    match = re.search(r"Yaw=([-+]?\d+\.?\d*)", line)

    if not match:
        return None

    return float(match.group(1))


def get_latest_yaw_from_lines(lines):
    latest_yaw = None

    for line in lines:
        yaw = parse_yaw(line)
        if yaw is not None:
            latest_yaw = yaw

    return latest_yaw


def read_yaw_for_short_time(ser, duration_s=0.45):
    """
    Reads Arduino serial output and returns latest yaw.
    """
    lines = read_available_lines(ser, duration_s)
    return get_latest_yaw_from_lines(lines)


# =========================
# Basic Manual Movements
# =========================

def manual_forward(ser, speed=DEFAULT_FORWARD_SPEED, duration_ms=DEFAULT_FORWARD_MS):
    send_command(ser, f"F {speed} {duration_ms}")
    read_available_lines(ser, duration_ms / 1000.0 + 0.2)


def manual_backward(ser, speed=DEFAULT_BACKWARD_SPEED, duration_ms=DEFAULT_BACKWARD_MS):
    send_command(ser, f"B {speed} {duration_ms}")
    read_available_lines(ser, duration_ms / 1000.0 + 0.2)


def manual_left(ser, speed=DEFAULT_TURN_SPEED, duration_ms=DEFAULT_TURN_MS):
    send_command(ser, f"L {speed} {duration_ms}")
    read_available_lines(ser, duration_ms / 1000.0 + 0.2)


def manual_right(ser, speed=DEFAULT_TURN_SPEED, duration_ms=DEFAULT_TURN_MS):
    send_command(ser, f"R {speed} {duration_ms}")
    read_available_lines(ser, duration_ms / 1000.0 + 0.2)


def manual_pid_forward_step(ser, speed=DEFAULT_FORWARD_SPEED, duration_ms=DEFAULT_FORWARD_MS):
    send_command(ser, f"P {speed} {duration_ms}")
    read_available_lines(ser, duration_ms / 1000.0 + 0.2)


def stop_robot(ser):
    send_command(ser, "S")
    read_available_lines(ser, 0.2)


# =========================
# Heading Correction Logic
# =========================

def correct_heading(ser, yaw):
    """
    If yaw is too large, stop and make a small corrective turn.
    """

    abs_yaw = abs(yaw)

    if abs_yaw < YAW_WARNING_DEG:
        return False

    print(f"Yaw drift detected: {yaw:.2f} deg")
    stop_robot(ser)
    time.sleep(0.1)

    if abs_yaw >= YAW_CRITICAL_DEG:
        correction_ms = BIG_CORRECTION_MS
        print("Critical yaw drift. Using big correction.")
    else:
        correction_ms = SMALL_CORRECTION_MS
        print("Small yaw drift. Using small correction.")

    if yaw > 0:
        turn_cmd = POSITIVE_YAW_CORRECTION_CMD
    else:
        turn_cmd = NEGATIVE_YAW_CORRECTION_CMD

    send_command(ser, f"{turn_cmd} {CORRECTION_SPEED} {correction_ms}")
    read_available_lines(ser, correction_ms / 1000.0 + 0.2)

    stop_robot(ser)
    time.sleep(0.1)

    print(f"Correction completed for yaw={yaw:.2f} deg")
    return True


def supervised_forward(ser, total_time_s, speed=DEFAULT_FORWARD_SPEED):
    """
    Move forward in short PID-controlled steps.
    Raspberry Pi watches Arduino yaw telemetry.
    If drift becomes too large, robot stops and corrects direction.
    """

    print()
    print("=== SUPERVISED FORWARD STARTED ===")
    print(f"Total time: {total_time_s:.1f} s")
    print(f"Speed: {speed}")
    print(f"Forward step: {FORWARD_STEP_MS} ms")
    print(f"Yaw warning: {YAW_WARNING_DEG} deg")
    print(f"Yaw critical: {YAW_CRITICAL_DEG} deg")
    print("Press CTRL+C to interrupt.")
    print("==================================")
    print()

    start_time = time.time()

    try:
        while time.time() - start_time < total_time_s:
            send_command(ser, f"P {speed} {FORWARD_STEP_MS}")

            yaw = read_yaw_for_short_time(
                ser,
                duration_s=(FORWARD_STEP_MS / 1000.0) + 0.15
            )

            if yaw is None:
                print("No yaw received from Arduino during this step.")
                continue

            print(f"Latest yaw: {yaw:.2f} deg")

            if abs(yaw) >= YAW_WARNING_DEG:
                correct_heading(ser, yaw)

        stop_robot(ser)
        print("=== SUPERVISED FORWARD FINISHED ===")

    except KeyboardInterrupt:
        stop_robot(ser)
        print("\nSupervised forward interrupted.")


# =========================
# User Input Parsing
# =========================

def print_menu():
    print()
    print("========== DeliveryBot Manual Controller ==========")
    print("Basic commands:")
    print("  f                 -> forward normal")
    print("  b                 -> backward")
    print("  l                 -> turn left")
    print("  r                 -> turn right")
    print("  p                 -> forward with Arduino PID once")
    print("  s                 -> stop")
    print()
    print("Commands with values:")
    print("  f speed ms         example: f 100 500")
    print("  b speed ms         example: b 100 500")
    print("  l speed ms         example: l 90 300")
    print("  r speed ms         example: r 90 300")
    print("  p speed ms         example: p 105 500")
    print()
    print("Supervised forward:")
    print("  sf seconds speed   example: sf 10 105")
    print("  sf                 default: 5 seconds, speed 105")
    print()
    print("Other:")
    print("  raw COMMAND        example: raw P 105 300")
    print("  q                 -> quit")
    print("===================================================")
    print()


def parse_speed_duration(parts, default_speed, default_duration):
    if len(parts) == 1:
        return default_speed, default_duration

    if len(parts) == 3:
        speed = int(parts[1])
        duration = int(parts[2])
        return speed, duration

    raise ValueError("Expected format: command OR command speed duration_ms")


def handle_user_command(ser, user_input):
    parts = user_input.strip().split()

    if not parts:
        return True

    cmd = parts[0].lower()

    if cmd == "q":
        stop_robot(ser)
        return False

    if cmd == "s":
        stop_robot(ser)
        return True

    if cmd == "f":
        speed, duration = parse_speed_duration(
            parts,
            DEFAULT_FORWARD_SPEED,
            DEFAULT_FORWARD_MS
        )
        manual_forward(ser, speed, duration)
        return True

    if cmd == "b":
        speed, duration = parse_speed_duration(
            parts,
            DEFAULT_BACKWARD_SPEED,
            DEFAULT_BACKWARD_MS
        )
        manual_backward(ser, speed, duration)
        return True

    if cmd == "l":
        speed, duration = parse_speed_duration(
            parts,
            DEFAULT_TURN_SPEED,
            DEFAULT_TURN_MS
        )
        manual_left(ser, speed, duration)
        return True

    if cmd == "r":
        speed, duration = parse_speed_duration(
            parts,
            DEFAULT_TURN_SPEED,
            DEFAULT_TURN_MS
        )
        manual_right(ser, speed, duration)
        return True

    if cmd == "p":
        speed, duration = parse_speed_duration(
            parts,
            DEFAULT_FORWARD_SPEED,
            DEFAULT_FORWARD_MS
        )
        manual_pid_forward_step(ser, speed, duration)
        return True

    if cmd == "sf":
        if len(parts) == 1:
            total_time_s = 5.0
            speed = DEFAULT_FORWARD_SPEED

        elif len(parts) == 3:
            total_time_s = float(parts[1])
            speed = int(parts[2])

        else:
            print("Expected: sf OR sf seconds speed")
            return True

        supervised_forward(ser, total_time_s, speed)
        return True

    if cmd == "raw":
        if len(parts) < 2:
            print("Expected: raw COMMAND")
            return True

        raw_command = " ".join(parts[1:])
        send_command(ser, raw_command)
        read_available_lines(ser, 0.5)
        return True

    print("Unknown command:", user_input)
    return True


# =========================
# Main
# =========================

def main():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {ARDUINO_PORT}: {e}")
        print("Try:")
        print("  sudo chmod 666 /dev/ttyUSB0")
        print("or permanent:")
        print("  sudo usermod -aG dialout $USER")
        print("  sudo reboot")
        return

    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected to Arduino motor controller.")
    print_menu()

    running = True

    try:
        while running:
            user_input = input("deliverybot> ").strip()

            try:
                running = handle_user_command(ser, user_input)
            except ValueError as e:
                print("Input error:", e)
            except Exception as e:
                print("Command error:", e)

    except KeyboardInterrupt:
        print("\nCTRL+C detected. Stopping robot.")
        stop_robot(ser)

    finally:
        stop_robot(ser)
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
