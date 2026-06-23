import serial
import time
import re

ARDUINO_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

FORWARD_SPEED = 105
FORWARD_STEP_MS = 300

CORRECTION_SPEED = 90
SMALL_CORRECTION_MS = 150
BIG_CORRECTION_MS = 250

YAW_WARNING_DEG = 4.0
YAW_CRITICAL_DEG = 7.0

TOTAL_RUN_TIME_S = 10.0

# If correction goes wrong direction, swap these two values
POSITIVE_YAW_CORRECTION_CMD = "R"
NEGATIVE_YAW_CORRECTION_CMD = "L"


def parse_yaw(line: str):
    """
    Expected Arduino line:
    PID,Yaw=4.75,Target=0.00,Error=-4.75,dt=0.020,Correction=-7.20
    """

    match = re.search(r"Yaw=([-+]?\d+\.?\d*)", line)

    if not match:
        return None

    return float(match.group(1))


def send_command(ser, command: str):
    print("SEND:", command)
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def read_yaw_for_short_time(ser, duration_s=0.35):
    """
    Reads Arduino serial output for a short time
    and returns the latest yaw value seen.
    """

    start = time.time()
    latest_yaw = None

    while time.time() - start < duration_s:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if line:
                print("ARDUINO:", line)

            yaw = parse_yaw(line)
            if yaw is not None:
                latest_yaw = yaw

        time.sleep(0.005)

    return latest_yaw


def correct_heading(ser, yaw):
    """
    If yaw is too large, stop and make a small correction turn.
    """

    abs_yaw = abs(yaw)

    if abs_yaw < YAW_WARNING_DEG:
        return

    send_command(ser, "S")
    time.sleep(0.1)

    if abs_yaw >= YAW_CRITICAL_DEG:
        correction_ms = BIG_CORRECTION_MS
    else:
        correction_ms = SMALL_CORRECTION_MS

    if yaw > 0:
        turn_cmd = POSITIVE_YAW_CORRECTION_CMD
    else:
        turn_cmd = NEGATIVE_YAW_CORRECTION_CMD

    command = f"{turn_cmd} {CORRECTION_SPEED} {correction_ms}"
    send_command(ser, command)

    time.sleep(correction_ms / 1000.0 + 0.2)

    send_command(ser, "S")
    time.sleep(0.1)

    print(f"Correction done for yaw={yaw:.2f} deg")


def main():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {ARDUINO_PORT}: {e}")
        print("Try:")
        print("  sudo chmod 666 /dev/ttyUSB0")
        return

    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected to Arduino.")
    print("Starting IMU-supervised forward movement.")
    print("Press CTRL+C to stop.")

    start_time = time.time()

    try:
        while time.time() - start_time < TOTAL_RUN_TIME_S:
            command = f"P {FORWARD_SPEED} {FORWARD_STEP_MS}"
            send_command(ser, command)

            yaw = read_yaw_for_short_time(
                ser,
                duration_s=(FORWARD_STEP_MS / 1000.0) + 0.15
            )

            if yaw is None:
                print("No yaw received from Arduino.")
                continue

            print(f"Latest yaw: {yaw:.2f} deg")

            if abs(yaw) >= YAW_WARNING_DEG:
                correct_heading(ser, yaw)

        send_command(ser, "S")
        print("Finished.")

    except KeyboardInterrupt:
        send_command(ser, "S")
        print("\nStopped by user.")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
