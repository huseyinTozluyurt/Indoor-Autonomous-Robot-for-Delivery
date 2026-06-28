import serial
import time
import re


# ============================================================
# Serial settings
# ============================================================

ARDUINO_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200


# ============================================================
# Long forward movement settings
# ============================================================

DEFAULT_SPEED = 180
DEFAULT_DURATION_S = 10.0

FORWARD_CHUNK_MS = 1000

MIN_SPEED = 80
MAX_SPEED = 255


# ============================================================
# Yaw / sapma thresholds
# ============================================================

YAW_DEADBAND_DEG = 5.0
YAW_CORRECTION_THRESHOLD_DEG = 10.0
YAW_CRITICAL_THRESHOLD_DEG = 18.0

# For today's test, use 1 so we can prove correction works.
# Later you can make it 2 or 3.
REQUIRED_BAD_YAW_COUNT = 1


# ============================================================
# Correction settings
# ============================================================

CORRECTION_SPEED = 240

SMALL_CORRECTION_MS = 350
MEDIUM_CORRECTION_MS = 550
BIG_CORRECTION_MS = 750

STOP_PAUSE_S = 0.20

# Since you observed right correction increased yaw:
# yaw > 0 -> turn left
# yaw < 0 -> turn right
POSITIVE_YAW_CORRECTION_CMD = "L"
NEGATIVE_YAW_CORRECTION_CMD = "R"


# ============================================================
# Arduino telemetry parsing
# ============================================================

def parse_yaw(line: str):
    match = re.search(r"Yaw=([-+]?\d+\.?\d*)", line)

    if not match:
        return None

    return float(match.group(1))


# ============================================================
# Serial helpers
# ============================================================

def send_command(ser, command: str):
    print(f"SEND: {command}")
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def read_lines_for_duration(ser, duration_s: float):
    lines = []
    start_time = time.time()

    while time.time() - start_time < duration_s:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line:
                print("ARDUINO:", line)
                lines.append(line)

        time.sleep(0.005)

    return lines


def get_latest_yaw(lines):
    latest_yaw = None

    for line in lines:
        yaw = parse_yaw(line)
        if yaw is not None:
            latest_yaw = yaw

    return latest_yaw


def stop_robot(ser):
    send_command(ser, "S")
    read_lines_for_duration(ser, 0.20)


# ============================================================
# Correction logic
# ============================================================

def choose_correction_duration(abs_yaw: float):
    if abs_yaw >= YAW_CRITICAL_THRESHOLD_DEG:
        return BIG_CORRECTION_MS

    if abs_yaw >= YAW_CORRECTION_THRESHOLD_DEG:
        return MEDIUM_CORRECTION_MS

    return SMALL_CORRECTION_MS


def correct_heading(ser, yaw: float):
    abs_yaw = abs(yaw)

    if abs_yaw < YAW_CORRECTION_THRESHOLD_DEG:
        return False

    print()
    print("========== HEADING CORRECTION ==========")
    print(f"Yaw drift detected: {yaw:.2f} deg")

    stop_robot(ser)
    time.sleep(STOP_PAUSE_S)

    correction_ms = choose_correction_duration(abs_yaw)

    if yaw > 0:
        turn_cmd = POSITIVE_YAW_CORRECTION_CMD
    else:
        turn_cmd = NEGATIVE_YAW_CORRECTION_CMD

    correction_command = f"{turn_cmd} {CORRECTION_SPEED} {correction_ms}"

    print(f"Correction command: {correction_command}")
    send_command(ser, correction_command)

    read_lines_for_duration(ser, correction_ms / 1000.0 + 0.35)

    stop_robot(ser)
    time.sleep(STOP_PAUSE_S)

    print("Correction completed.")
    print("========================================")
    print()

    return True


# ============================================================
# Long forward supervised movement
# ============================================================

def long_forward_supervised(ser, speed: int, duration_s: float):
    speed = max(MIN_SPEED, min(MAX_SPEED, speed))

    print()
    print("=================================================")
    print("LONG FORWARD IMU-SUPERVISED MOVEMENT STARTED")
    print(f"Speed: {speed}")
    print(f"Duration: {duration_s:.2f} seconds")
    print(f"Forward chunk: {FORWARD_CHUNK_MS} ms")
    print(f"Yaw deadband: {YAW_DEADBAND_DEG} deg")
    print(f"Correction threshold: {YAW_CORRECTION_THRESHOLD_DEG} deg")
    print(f"Critical threshold: {YAW_CRITICAL_THRESHOLD_DEG} deg")
    print(f"Required bad yaw count: {REQUIRED_BAD_YAW_COUNT}")
    print(f"Correction speed: {CORRECTION_SPEED}")
    print("=================================================")
    print()

    # Critical: reset yaw only once at the start of A-to-B movement.
    send_command(ser, "Z")
    read_lines_for_duration(ser, 0.40)

    start_time = time.time()
    end_time = start_time + duration_s

    correction_count = 0
    bad_yaw_count = 0
    last_yaw = None

    try:
        while time.time() < end_time:
            remaining_s = end_time - time.time()

            if remaining_s <= 0:
                break

            chunk_ms = min(FORWARD_CHUNK_MS, int(remaining_s * 1000))

            if chunk_ms <= 0:
                break

            # Use G, not P.
            # G = PID forward without yaw reset.
            command = f"G {speed} {chunk_ms}"
            send_command(ser, command)

            lines = read_lines_for_duration(
                ser,
                duration_s=(chunk_ms / 1000.0) + 0.25
            )

            yaw = get_latest_yaw(lines)

            if yaw is None:
                print("WARNING: No yaw data received from Arduino.")
                continue

            last_yaw = yaw
            abs_yaw = abs(yaw)

            print(f"Latest yaw: {yaw:.2f} deg")

            if abs_yaw <= YAW_DEADBAND_DEG:
                print("Yaw inside deadband. No correction.")
                bad_yaw_count = 0
                continue

            if abs_yaw < YAW_CORRECTION_THRESHOLD_DEG:
                print("Yaw drift acceptable. Arduino PID should handle it.")
                bad_yaw_count = 0
                continue

            bad_yaw_count += 1

            print(
                f"Yaw above correction threshold. "
                f"Bad yaw count: {bad_yaw_count}/{REQUIRED_BAD_YAW_COUNT}"
            )

            if bad_yaw_count < REQUIRED_BAD_YAW_COUNT:
                print("Waiting before correction.")
                continue

            corrected = correct_heading(ser, yaw)

            if corrected:
                correction_count += 1
                bad_yaw_count = 0

        stop_robot(ser)

        print()
        print("=================================================")
        print("LONG FORWARD FINISHED")
        print(f"Total corrections: {correction_count}")

        if last_yaw is not None:
            print(f"Last yaw observed: {last_yaw:.2f} deg")

        print("=================================================")
        print()

    except KeyboardInterrupt:
        print("\nCTRL+C detected during long forward.")
        stop_robot(ser)


# ============================================================
# Manual command interface
# ============================================================

def print_menu():
    print()
    print("=============== DeliveryBot Long Forward Controller ===============")
    print("Main command:")
    print("  lf speed seconds")
    print("  Example: lf 160 10")
    print("  Example: lf 180 10")
    print("  Example: lf 200 10")
    print()
    print("Manual commands:")
    print("  f speed ms       -> normal forward")
    print("  b speed ms       -> backward")
    print("  l speed ms       -> turn left")
    print("  r speed ms       -> turn right")
    print("  p speed ms       -> PID forward with yaw reset")
    print("  g speed ms       -> PID forward without yaw reset")
    print("  z                -> reset yaw")
    print("  s                -> stop")
    print("  raw COMMAND      -> send raw Arduino command")
    print("  q                -> quit")
    print("===================================================================")
    print()


def handle_user_input(ser, user_input: str):
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

    if cmd == "z":
        send_command(ser, "Z")
        read_lines_for_duration(ser, 0.40)
        return True

    if cmd == "lf":
        if len(parts) == 1:
            speed = DEFAULT_SPEED
            duration_s = DEFAULT_DURATION_S

        elif len(parts) == 3:
            speed = int(parts[1])
            duration_s = float(parts[2])

        else:
            print("Usage: lf speed seconds")
            print("Example: lf 180 10")
            return True

        long_forward_supervised(ser, speed, duration_s)
        return True

    if cmd in ["f", "b", "l", "r", "p", "g"]:
        if len(parts) != 3:
            print(f"Usage: {cmd} speed duration_ms")
            print(f"Example: {cmd} 180 1000")
            return True

        speed = int(parts[1])
        duration_ms = int(parts[2])

        arduino_cmd = cmd.upper()
        send_command(ser, f"{arduino_cmd} {speed} {duration_ms}")
        read_lines_for_duration(ser, duration_ms / 1000.0 + 0.25)
        return True

    if cmd == "raw":
        if len(parts) < 2:
            print("Usage: raw COMMAND")
            print("Example: raw G 180 1000")
            return True

        raw_command = " ".join(parts[1:])
        send_command(ser, raw_command)
        read_lines_for_duration(ser, 0.60)
        return True

    print("Unknown command:", user_input)
    return True


# ============================================================
# Main
# ============================================================

def main():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {ARDUINO_PORT}: {e}")
        print()
        print("Try:")
        print("  sudo chmod 666 /dev/ttyUSB0")
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
                running = handle_user_input(ser, user_input)
            except ValueError as e:
                print("Input value error:", e)
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
