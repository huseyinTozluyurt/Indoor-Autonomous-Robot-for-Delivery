#!/usr/bin/env python3

# ======================================================
# DeliveryBot Raspberry Pi Serial Motor Controller
# With file-guided lateral correction
#
# Arduino command format:
#   F left_speed right_speed duration_ms
#   B left_speed right_speed duration_ms
#   L left_speed right_speed duration_ms
#   R left_speed right_speed duration_ms
#   S
#
# New correction idea:
#   Raspberry Pi reads lateral_error.txt every movement chunk.
#
# File examples:
#   -15.0
#   lateral_error_cm=-15.0
#
# Convention:
#   lateral_error_cm < 0  -> robot is too LEFT  -> turn RIGHT
#   lateral_error_cm > 0  -> robot is too RIGHT -> turn LEFT
#
# The robot moves in short chunks, for example 1000 ms.
# Correction stop/turn/pause time is NOT counted as forward/backward drive time.
# ======================================================

import os
import serial
import time
import sys


DEFAULT_ERROR_FILE = "/home/huseyin/delivery_bot_serial/updated/lateral_error.txt"


class DeliveryBotMotorController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )

            print("[INFO] Waiting for Arduino boot & IMU calibration (4 seconds)...")
            time.sleep(4.0)

            # Clear old serial data accumulated during boot
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            print(f"[OK] Connected to Arduino on {self.port}")
            self.read_available_lines(verbose=True)

        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {self.port}")
            print(e)
            sys.exit(1)

    def close(self):
        if self.ser and self.ser.is_open:
            self.stop()
            time.sleep(0.1)
            self.ser.close()
            print("[OK] Serial connection closed.")

    def send_command(self, command, read_after=True):
            if not self.ser or not self.ser.is_open:
                print("[ERROR] Serial port is not open.")
                return

            command = command.strip() + "\n"
            self.ser.write(command.encode("utf-8"))
            time.sleep(0.002)  # Short delay to prevent UART byte fragmentation
            self.ser.flush()

            print(f"[TX] {command.strip()}")

            if read_after:
                time.sleep(0.03)
                self.read_available_lines(verbose=True)


    def read_available_lines(self, verbose=False):
        """
        Reads and flushes lines from Arduino.
        Filters out high-frequency 'IMU yaw=' telemetry to prevent terminal lag,
        unless verbose is explicitly set to True.
        """
        if not self.ser:
            return

        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    # Filter out repetitive IMU yaw logs during movement
                    if line.startswith("IMU yaw=") and not verbose:
                        continue
                    print(f"[ARDUINO] {line}")
            except Exception as e:
                print(f"[WARN] Could not read line: {e}")
                break

    @staticmethod
    def clamp_speed(value):
        value = int(value)
        return max(0, min(255, value))

    @staticmethod
    def clamp_duration(value):
        value = int(value)
        return max(0, value)

    def move_forward(self, left_speed, right_speed, duration_ms):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)
        duration_ms = self.clamp_duration(duration_ms)
        self.send_command(f"F {left_speed} {right_speed} {duration_ms}")

    def move_backward(self, left_speed, right_speed, duration_ms):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)
        duration_ms = self.clamp_duration(duration_ms)
        self.send_command(f"B {left_speed} {right_speed} {duration_ms}")

    def turn_left(self, left_speed, right_speed, duration_ms):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)
        duration_ms = self.clamp_duration(duration_ms)
        self.send_command(f"L {left_speed} {right_speed} {duration_ms}")

    def turn_right(self, left_speed, right_speed, duration_ms):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)
        duration_ms = self.clamp_duration(duration_ms)
        self.send_command(f"R {left_speed} {right_speed} {duration_ms}")

    def reset_yaw(self):
            """Sends Z command to reset IMU yaw offset on Arduino."""
            self.send_command("Z", read_after=True)

    def stop(self):
        """Sends Stop command and ensures Arduino acknowledges it."""
        if not self.ser or not self.ser.is_open:
            return

        # Send stop command twice to guarantee arrival over UART
        self.send_command("S", read_after=False)
        time.sleep(0.05)
        self.send_command("S", read_after=True)


def clamp_value(value, min_value, max_value):
    value = int(value)
    return max(min_value, min(max_value, value))


def calculate_left_right_speeds(base_power, difference):
    left_speed = base_power + difference
    right_speed = base_power - difference
    left_speed = clamp_value(left_speed, 0, 255)
    right_speed = clamp_value(right_speed, 0, 255)
    return left_speed, right_speed


def ensure_lateral_error_file(path=DEFAULT_ERROR_FILE):
    parent_dir = os.path.dirname(path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", encoding="utf-8") as f:
            f.write("lateral_error_cm=0.0\n")


def write_lateral_error_cm(value_cm, path=DEFAULT_ERROR_FILE):
    parent_dir = os.path.dirname(path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        f.write(f"lateral_error_cm={float(value_cm):.2f}\n")


def read_lateral_error_cm(path=DEFAULT_ERROR_FILE):
    try:
        if not os.path.exists(path):
            ensure_lateral_error_file(path)
            return 0.0

        with open(path, "r", encoding="utf-8") as f:
            text = f.read().strip()

        if not text:
            write_lateral_error_cm(0.0, path)
            return 0.0

        lines = [line.strip() for line in text.splitlines() if line.strip()]
        if not lines:
            return 0.0

        line = lines[0]
        if "=" in line:
            line = line.split("=", 1)[1].strip()

        return float(line)

    except Exception as e:
        print(f"[WARN] Could not read lateral error from {path}: {e}")
        return 0.0


def calculate_correction_turn_ms(abs_error_cm, threshold_cm, ms_per_cm, min_turn_ms, max_turn_ms):
    excess_cm = max(0.0, abs_error_cm - threshold_cm)
    raw_ms = min_turn_ms + (excess_cm * ms_per_cm)
    return int(max(min_turn_ms, min(max_turn_ms, raw_ms)))


def run_file_guided_movement(robot, direction, params):


    completed_drive_ms = 0
    correction_count = 0

    ensure_lateral_error_file(error_file)

    # 1. ALWAYS RESET YAW TO ZERO AT START OF MOVEMENT RUN
    print("\n[INFO] Resetting IMU yaw baseline to 0.00°...")
    robot.reset_yaw()
    time.sleep(0.1)

    print("\n[FILE-GUIDED] Starting movement")
    print(f"[FILE-GUIDED] Direction: {direction}")
    print(f"[FILE-GUIDED] Drive L/R: {drive_left}/{drive_right}")
    # ... rest of loop code ...


    drive_left = params["drive_left"]
    drive_right = params["drive_right"]
    total_duration_ms = params["total_duration_ms"]
    error_file = params["error_file"]
    threshold_cm = params["threshold_cm"]
    chunk_ms = params["chunk_ms"]

    correction_wait_s = params["correction_wait_s"]
    correction_pause_s = params["correction_pause_s"]

    turn_left_speed = params["turn_left_speed"]
    turn_right_speed = params["turn_right_speed"]
    turn_ms_per_cm = params["turn_ms_per_cm"]
    min_turn_ms = params["min_turn_ms"]
    max_turn_ms = params["max_turn_ms"]
    reset_after_correction = params.get("reset_after_correction", True)

    completed_drive_ms = 0
    correction_count = 0

    ensure_lateral_error_file(error_file)

    print("\n[FILE-GUIDED] Starting movement")
    print(f"[FILE-GUIDED] Direction: {direction}")
    print(f"[FILE-GUIDED] Drive L/R: {drive_left}/{drive_right}")
    print(f"[FILE-GUIDED] Total drive duration: {total_duration_ms} ms")
    print(f"[FILE-GUIDED] Chunk: {chunk_ms} ms\n")

    try:
        while completed_drive_ms < total_duration_ms:
            robot.read_available_lines(verbose=False)

            error_cm = read_lateral_error_cm(error_file)
            abs_error_cm = abs(error_cm)

            remaining_ms = total_duration_ms - completed_drive_ms
            current_chunk_ms = min(chunk_ms, remaining_ms)

            print(
                f"[FILE-GUIDED] done={completed_drive_ms}/{total_duration_ms} ms | "
                f"error={error_cm:.2f} cm | threshold=±{threshold_cm:.2f} cm"
            )

            if abs_error_cm >= threshold_cm:
                correction_count += 1
                turn_ms = calculate_correction_turn_ms(
                    abs_error_cm, threshold_cm, turn_ms_per_cm, min_turn_ms, max_turn_ms
                )

                if error_cm < 0:
                    correction_cmd = "R"
                    correction_text = "Robot too LEFT -> turning RIGHT"
                else:
                    correction_cmd = "L"
                    correction_text = "Robot too RIGHT -> turning LEFT"

                print(f"\n[CORRECTION_TRIGGER] {correction_text}")
                print(f"[CORRECTION] error={error_cm:.2f} cm, turn_ms={turn_ms}")
                robot.stop()

                time.sleep(correction_wait_s)
                robot.read_available_lines(verbose=True)

                if correction_cmd == "R":
                    robot.turn_right(turn_left_speed, turn_right_speed, turn_ms)
                else:
                    robot.turn_left(turn_left_speed, turn_right_speed, turn_ms)

                time.sleep((turn_ms / 1000.0) + correction_pause_s)
                robot.read_available_lines(verbose=True)

                if reset_after_correction:
                    write_lateral_error_cm(0.0, error_file)

                continue

                # Execute movement chunk
                if direction == "F":
                    robot.move_forward(drive_left, drive_right, current_chunk_ms)
                elif direction == "B":
                    robot.move_backward(drive_left, drive_right, current_chunk_ms)

                # Wait during chunk execution
                sleep_end = time.time() + (current_chunk_ms / 1000.0)
                while time.time() < sleep_end:
                    robot.read_available_lines(verbose=False)
                    time.sleep(0.02)

                # UPDATE AND PRINT PROGRESS ACCURATELY
                completed_drive_ms += current_chunk_ms
                print(f"[FILE-GUIDED] Chunk finished | Progress: {completed_drive_ms}/{total_duration_ms} ms")



    except KeyboardInterrupt:
        print("\n[FILE-GUIDED] Interrupted by user. Stopping robot.")
        robot.stop()





def print_menu():
    print()
    print("========== DeliveryBot Motor Controller ==========")
    print("1 - Forward")
    print("2 - Backward")
    print("3 - Turn Left")
    print("4 - Turn Right")
    print("5 - Stop")
    print("6 - Test sequence")
    print("7 - File-guided Forward correction")
    print("8 - File-guided Backward correction")
    print("9 - Create/reset lateral_error.txt")
    print("10 - Manually set lateral_error.txt")
    print("0 - Exit")
    print("=================================================")


def clamp_value(value, min_value, max_value):
    value = int(value)

    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


def calculate_left_right_speeds(base_power, difference):
    """
    base_power:
        Main PWM power value for both sides.

    difference:
        Positive difference:
            left motor side becomes stronger
            right motor side becomes weaker

        Negative difference:
            left motor side becomes weaker
            right motor side becomes stronger

    Formula:
        left_speed  = base_power + difference
        right_speed = base_power - difference
    """

    left_speed = base_power + difference
    right_speed = base_power - difference

    left_speed = clamp_value(left_speed, 0, 255)
    right_speed = clamp_value(right_speed, 0, 255)

    return left_speed, right_speed


def get_power_difference_duration():
    try:
        print()
        print("Enter movement parameters:")
        print("Base power: normal motor power, usually 100-220")
        print("Difference: left-right correction")
        print()
        print("Example:")
        print("  base_power = 200")
        print("  difference = 0   -> left=200, right=200")
        print("  difference = 10  -> left=210, right=190")
        print("  difference = -10 -> left=190, right=210")
        print()

        base_power = int(input("Base motor power [0-255]: "))
        difference = int(input("Left-right difference [-255 to 255]: "))
        duration_ms = int(input("Duration in ms: "))

        base_power = clamp_value(base_power, 0, 255)
        difference = clamp_value(difference, -255, 255)
        duration_ms = clamp_value(duration_ms, 0, 60000)

        left_speed, right_speed = calculate_left_right_speeds(
            base_power,
            difference
        )

        print()
        print(f"[CALC] Base power: {base_power}")
        print(f"[CALC] Difference: {difference}")
        print(f"[CALC] Left speed:  {left_speed}")
        print(f"[CALC] Right speed: {right_speed}")
        print(f"[CALC] Duration:    {duration_ms} ms")
        print()

        return left_speed, right_speed, duration_ms

    except ValueError:
        print("[ERROR] Please enter valid integer numbers.")
        return None, None, None


def ensure_lateral_error_file(path=DEFAULT_ERROR_FILE):
    """
    Ensures the default lateral error file exists and contains a valid default value.
    """
    parent_dir = os.path.dirname(path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, "w", encoding="utf-8") as f:
            f.write("lateral_error_cm=0.0\n")
        print(f"[INFO] Initialized {path} with lateral_error_cm=0.0")


def write_lateral_error_cm(value_cm, path=DEFAULT_ERROR_FILE):
    parent_dir = os.path.dirname(path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        f.write(f"lateral_error_cm={float(value_cm):.2f}\n")


def create_or_reset_lateral_error_file(path=DEFAULT_ERROR_FILE):
    write_lateral_error_cm(0.0, path)

    print(f"[OK] Created/reset {path}")
    print("[INFO] Edit this file while the robot is running:")
    print("       lateral_error_cm=-15.0  -> robot too LEFT, correction turns RIGHT")
    print("       lateral_error_cm=15.0   -> robot too RIGHT, correction turns LEFT")
    print("       lateral_error_cm=0.0    -> no correction")


def manually_set_lateral_error_file(path=DEFAULT_ERROR_FILE):
    try:
        ensure_lateral_error_file(path)

        print()
        print("Manual lateral error setting")
        print("----------------------------")
        print("negative value -> robot too LEFT  -> correction turns RIGHT")
        print("positive value -> robot too RIGHT -> correction turns LEFT")
        print("0              -> no correction")
        print()
        value = float(input("Enter lateral_error_cm value: "))

        write_lateral_error_cm(value, path)

        print(f"[OK] Wrote lateral_error_cm={value:.2f} to {path}")

    except ValueError:
        print("[ERROR] Invalid number.")


def read_lateral_error_cm(path=DEFAULT_ERROR_FILE):
    """
    Reads lateral error from a text file.

    Supported formats:
        -12.5
        lateral_error_cm=-12.5

    Convention:
        negative value -> robot is too left
        positive value -> robot is too right
    """

    try:
        if not os.path.exists(path):
            print(f"[WARN] {path} does not exist. Creating it with 0.0 cm.")
            ensure_lateral_error_file(path)
            return 0.0

        with open(path, "r", encoding="utf-8") as f:
            text = f.read().strip()

        if not text:
            print(f"[WARN] {path} is empty. Reinitializing with 0.0 cm.")
            write_lateral_error_cm(0.0, path)
            return 0.0

        # Allow comments/extra lines; use first meaningful line.
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        if not lines:
            return 0.0

        line = lines[0]

        if "=" in line:
            line = line.split("=", 1)[1].strip()

        return float(line)

    except Exception as e:
        print(f"[WARN] Could not read lateral error from {path}: {e}")
        return 0.0


def get_file_guided_params():
    try:
        print()
        print("File-guided movement parameters")
        print("--------------------------------")
        print("The robot will move in short chunks.")
        print("Before each chunk, it reads lateral_error.txt.")
        print()
        print("lateral_error_cm < 0  -> robot too LEFT  -> turn RIGHT")
        print("lateral_error_cm > 0  -> robot too RIGHT -> turn LEFT")
        print()

        base_power = int(input("Forward/backward base power [0-255, recommended 200-220]: "))
        difference = int(input("Forward/backward difference [-255 to 255, usually 0 if Arduino trim handles it]: "))
        total_duration_ms = int(input("Total drive duration in ms: "))

        # Always use the default lateral error file defined at the top.
        # This prevents accidentally entering a directory instead of a .txt file.
        error_file = DEFAULT_ERROR_FILE
        ensure_lateral_error_file(error_file)
        print(f"[INFO] Using lateral error file: {error_file}")

        threshold_cm = float(input("Lateral threshold in cm [example 10]: ") or "10")
        chunk_ms = int(input("Movement chunk duration in ms [example 1000]: ") or "1000")

        correction_wait_s = float(input("Stop/wait before correction turn in seconds [example 2.0]: ") or "2.0")
        correction_pause_s = float(input("Pause after correction turn in seconds [example 0.3]: ") or "0.3")

        turn_base_power = int(input("Correction turn base power [example 200]: ") or "200")
        turn_difference = int(input("Correction turn difference [-255 to 255, usually 0]: ") or "0")
        turn_ms_per_cm = float(input("Correction turn ms per cm over threshold [example 15]: ") or "15")
        min_turn_ms = int(input("Minimum correction turn ms [example 160]: ") or "160")
        max_turn_ms = int(input("Maximum correction turn ms [example 500]: ") or "500")

        reset_after_correction_text = input("Reset lateral_error.txt to 0 after correction? [Y/n]: ").strip().lower()
        reset_after_correction = reset_after_correction_text not in ("n", "no")

        base_power = clamp_value(base_power, 0, 255)
        difference = clamp_value(difference, -255, 255)
        total_duration_ms = clamp_value(total_duration_ms, 0, 600000)
        chunk_ms = clamp_value(chunk_ms, 100, 5000)
        turn_base_power = clamp_value(turn_base_power, 0, 255)
        turn_difference = clamp_value(turn_difference, -255, 255)
        min_turn_ms = clamp_value(min_turn_ms, 0, 5000)
        max_turn_ms = clamp_value(max_turn_ms, min_turn_ms, 5000)

        drive_left, drive_right = calculate_left_right_speeds(base_power, difference)
        turn_left_speed, turn_right_speed = calculate_left_right_speeds(turn_base_power, turn_difference)

        return {
            "drive_left": drive_left,
            "drive_right": drive_right,
            "total_duration_ms": total_duration_ms,
            "error_file": error_file,
            "threshold_cm": abs(threshold_cm),
            "chunk_ms": chunk_ms,
            "correction_wait_s": correction_wait_s,
            "correction_pause_s": correction_pause_s,
            "turn_left_speed": turn_left_speed,
            "turn_right_speed": turn_right_speed,
            "turn_ms_per_cm": turn_ms_per_cm,
            "min_turn_ms": min_turn_ms,
            "max_turn_ms": max_turn_ms,
            "reset_after_correction": reset_after_correction,
        }

    except ValueError:
        print("[ERROR] Invalid input.")
        return None


def calculate_correction_turn_ms(abs_error_cm, threshold_cm, ms_per_cm, min_turn_ms, max_turn_ms):
    """
    Correction turn duration is proportional to how far the robot is beyond the threshold.

    Example:
        abs_error_cm = 15 cm
        threshold_cm = 10 cm
        excess = 5 cm
        ms_per_cm = 15
        raw turn = 75 ms
        constrained by min_turn_ms, max_turn_ms
    """

    excess_cm = max(0.0, abs_error_cm - threshold_cm)
    raw_ms = min_turn_ms + (excess_cm * ms_per_cm)

    if raw_ms < min_turn_ms:
        raw_ms = min_turn_ms

    if raw_ms > max_turn_ms:
        raw_ms = max_turn_ms

    return int(raw_ms)


def run_file_guided_movement(robot, direction, params):
    """
    direction:
        "F" for forward
        "B" for backward

    Important:
        completed_drive_ms counts only actual forward/backward movement chunks.
        Stop/wait/turn/pause time does not reduce the requested drive duration.
    """

    drive_left = params["drive_left"]
    drive_right = params["drive_right"]
    total_duration_ms = params["total_duration_ms"]
    error_file = params["error_file"]
    threshold_cm = params["threshold_cm"]
    chunk_ms = params["chunk_ms"]

    correction_wait_s = params["correction_wait_s"]
    correction_pause_s = params["correction_pause_s"]

    turn_left_speed = params["turn_left_speed"]
    turn_right_speed = params["turn_right_speed"]
    turn_ms_per_cm = params["turn_ms_per_cm"]
    min_turn_ms = params["min_turn_ms"]
    max_turn_ms = params["max_turn_ms"]
    reset_after_correction = params.get("reset_after_correction", True)

    completed_drive_ms = 0
    correction_count = 0

    ensure_lateral_error_file(error_file)

    print()
    print("[FILE-GUIDED] Starting movement")
    print(f"[FILE-GUIDED] Direction: {direction}")
    print(f"[FILE-GUIDED] Drive L/R: {drive_left}/{drive_right}")
    print(f"[FILE-GUIDED] Total drive duration: {total_duration_ms} ms")
    print(f"[FILE-GUIDED] Chunk: {chunk_ms} ms")
    print(f"[FILE-GUIDED] Error file: {error_file}")
    print(f"[FILE-GUIDED] Threshold: ±{threshold_cm:.2f} cm")
    print()

    try:
        while completed_drive_ms < total_duration_ms:
            robot.read_available_lines()

            error_cm = read_lateral_error_cm(error_file)
            abs_error_cm = abs(error_cm)

            remaining_ms = total_duration_ms - completed_drive_ms
            current_chunk_ms = min(chunk_ms, remaining_ms)

            print(
                f"[FILE-GUIDED] done={completed_drive_ms}/{total_duration_ms} ms | "
                f"error={error_cm:.2f} cm | threshold=±{threshold_cm:.2f} cm"
            )

            if abs_error_cm >= threshold_cm:
                correction_count += 1

                turn_ms = calculate_correction_turn_ms(
                    abs_error_cm=abs_error_cm,
                    threshold_cm=threshold_cm,
                    ms_per_cm=turn_ms_per_cm,
                    min_turn_ms=min_turn_ms,
                    max_turn_ms=max_turn_ms
                )

                # Convention:
                # error < 0: robot too LEFT, turn RIGHT.
                # error > 0: robot too RIGHT, turn LEFT.
                if error_cm < 0:
                    correction_cmd = "R"
                    correction_text = "Robot too LEFT -> turning RIGHT"
                else:
                    correction_cmd = "L"
                    correction_text = "Robot too RIGHT -> turning LEFT"

                print()
                print("[CORRECTION_TRIGGER]")
                print(f"[CORRECTION] {correction_text}")
                print(f"[CORRECTION] error={error_cm:.2f} cm, turn_ms={turn_ms}")
                print(f"[CORRECTION] correction_count={correction_count}")
                print("[CORRECTION] stopping...")
                robot.stop()

                print(f"[CORRECTION] waiting {correction_wait_s:.2f} seconds before turn...")
                time.sleep(correction_wait_s)
                robot.read_available_lines()

                if correction_cmd == "R":
                    robot.turn_right(turn_left_speed, turn_right_speed, turn_ms)
                else:
                    robot.turn_left(turn_left_speed, turn_right_speed, turn_ms)

                time.sleep((turn_ms / 1000.0) + correction_pause_s)
                robot.read_available_lines()

                if reset_after_correction:
                    write_lateral_error_cm(0.0, error_file)
                    print("[CORRECTION] lateral_error.txt reset to 0.0 after correction.")

                print("[CORRECTION] correction finished; continuing movement.")
                print()

                # Do not add correction time to completed_drive_ms.
                continue

            # No correction needed: move one chunk.
            if direction == "F":
                robot.move_forward(drive_left, drive_right, current_chunk_ms)
            elif direction == "B":
                robot.move_backward(drive_left, drive_right, current_chunk_ms)
            else:
                print(f"[ERROR] Unsupported direction: {direction}")
                robot.stop()
                return

            # Sleep in short increments to flush serial telemetry continuously
            sleep_end = time.time() + (current_chunk_ms / 1000.0)
            while time.time() < sleep_end:
                robot.read_available_lines()
                time.sleep(0.05)

            completed_drive_ms += current_chunk_ms

        robot.stop()
        print("[FILE-GUIDED] Movement complete.")

    except KeyboardInterrupt:
        print()
        print("[FILE-GUIDED] Keyboard interrupt. Stopping robot.")
        robot.stop()


def run_test_sequence(robot):
    print()
    print("[TEST] Starting basic movement test...")
    print("[TEST] This test also uses base power and difference.")
    print()

    forward_base_power = 140
    forward_difference = 0

    backward_base_power = 140
    backward_difference = 0

    left_turn_base_power = 140
    left_turn_difference = 0

    right_turn_base_power = 140
    right_turn_difference = 0

    duration = 1000
    pause = 1.0

    forward_left, forward_right = calculate_left_right_speeds(
        forward_base_power,
        forward_difference
    )

    print("[TEST] Forward")
    print(f"[TEST] Left={forward_left}, Right={forward_right}, Duration={duration}")
    robot.move_forward(forward_left, forward_right, duration)
    time.sleep(duration / 1000.0 + pause)

    backward_left, backward_right = calculate_left_right_speeds(
        backward_base_power,
        backward_difference
    )

    print("[TEST] Backward")
    print(f"[TEST] Left={backward_left}, Right={backward_right}, Duration={duration}")
    robot.move_backward(backward_left, backward_right, duration)
    time.sleep(duration / 1000.0 + pause)

    left_turn_left, left_turn_right = calculate_left_right_speeds(
        left_turn_base_power,
        left_turn_difference
    )

    print("[TEST] Turn Left")
    print(f"[TEST] Left={left_turn_left}, Right={left_turn_right}, Duration={duration}")
    robot.turn_left(left_turn_left, left_turn_right, duration)
    time.sleep(duration / 1000.0 + pause)

    right_turn_left, right_turn_right = calculate_left_right_speeds(
        right_turn_base_power,
        right_turn_difference
    )

    print("[TEST] Turn Right")
    print(f"[TEST] Left={right_turn_left}, Right={right_turn_right}, Duration={duration}")
    robot.turn_right(right_turn_left, right_turn_right, duration)
    time.sleep(duration / 1000.0 + pause)

    print("[TEST] Stop")
    robot.stop()


def main():
    port = "/dev/ttyUSB0"

    # You can also run like:
    # python3 deliverybot_file_guided_controller.py /dev/ttyUSB0
    if len(sys.argv) >= 2:
        port = sys.argv[1]

    robot = DeliveryBotMotorController(port=port)
    robot.connect()

    try:
        while True:
            print_menu()
            choice = input("Select option: ").strip()

            if choice == "1":
                left, right, duration = get_power_difference_duration()
                if left is not None:
                    robot.move_forward(left, right, duration)

            elif choice == "2":
                left, right, duration = get_power_difference_duration()
                if left is not None:
                    robot.move_backward(left, right, duration)

            elif choice == "3":
                left, right, duration = get_power_difference_duration()
                if left is not None:
                    robot.turn_left(left, right, duration)

            elif choice == "4":
                left, right, duration = get_power_difference_duration()
                if left is not None:
                    robot.turn_right(left, right, duration)

            elif choice == "5":
                robot.stop()

            elif choice == "6":
                run_test_sequence(robot)

            elif choice == "7":
                params = get_file_guided_params()
                if params:
                    run_file_guided_movement(robot, direction="F", params=params)

            elif choice == "8":
                params = get_file_guided_params()
                if params:
                    run_file_guided_movement(robot, direction="B", params=params)

            elif choice == "9":
                create_or_reset_lateral_error_file(DEFAULT_ERROR_FILE)

            elif choice == "10":
                manually_set_lateral_error_file(DEFAULT_ERROR_FILE)

            elif choice == "0":
                print("[INFO] Exiting...")
                break

            else:
                print("[ERROR] Unknown option.")

            robot.read_available_lines()

    except KeyboardInterrupt:
        print()
        print("[INFO] Keyboard interrupt received.")

    finally:
        robot.close()


if __name__ == "__main__":
    main()
