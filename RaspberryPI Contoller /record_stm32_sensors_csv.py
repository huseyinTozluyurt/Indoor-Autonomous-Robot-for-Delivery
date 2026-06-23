import serial
import time
import csv
from datetime import datetime
from pathlib import Path

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

OUTPUT_DIR = Path("sensor_logs")
OUTPUT_DIR.mkdir(exist_ok=True)

timestamp_for_filename = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
CSV_FILE = OUTPUT_DIR / f"sensor_log_{timestamp_for_filename}.csv"


def parse_sensor_line(line: str):
    """
    Expected STM32 format:
    front,left,right,back,error

    Example:
    293,10,16,11,0
    """

    parts = line.split(",")

    if len(parts) != 5:
        raise ValueError(f"Expected 5 values, got {len(parts)}")

    front = int(parts[0])
    left = int(parts[1])
    right = int(parts[2])
    back = int(parts[3])
    error = int(parts[4])

    return front, left, right, back, error


def main():
    print(f"Opening serial port: {SERIAL_PORT}")
    print(f"Baud rate: {BAUD_RATE}")
    print(f"CSV output file: {CSV_FILE}")

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    except serial.SerialException as e:
        print(f"Could not open {SERIAL_PORT}: {e}")
        print()
        print("Fix permission with:")
        print("  sudo usermod -aG dialout $USER")
        print("  sudo reboot")
        print()
        print("Temporary test fix:")
        print("  sudo chmod 666 /dev/ttyS0")
        return

    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected.")
    print("Recording STM32 distance sensor data...")
    print("Press CTRL+C to stop recording.")
    print("-" * 70)

    start_time = time.time()

    with open(CSV_FILE, mode="w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "timestamp",
            "elapsed_time_s",
            "front_cm",
            "left_cm",
            "right_cm",
            "back_cm",
            "error"
        ])

        while True:
            try:
                raw = ser.readline()

                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()

                if not line or "," not in line:
                    continue

                try:
                    front, left, right, back, error = parse_sensor_line(line)
                except ValueError as e:
                    print("Invalid line:", line, "|", e)
                    continue

                current_time = time.time()
                elapsed_time = current_time - start_time
                timestamp = datetime.now().isoformat(timespec="milliseconds")

                writer.writerow([
                    timestamp,
                    round(elapsed_time, 3),
                    front,
                    left,
                    right,
                    back,
                    error
                ])

                # Force data to be written to disk immediately
                file.flush()

                print(
                    f"{timestamp} | "
                    f"Front: {front:3d} cm | "
                    f"Left: {left:3d} cm | "
                    f"Right: {right:3d} cm | "
                    f"Back: {back:3d} cm | "
                    f"Error: {error}"
                )

            except KeyboardInterrupt:
                print("\nRecording stopped.")
                print(f"CSV saved to: {CSV_FILE}")
                break

            except Exception as e:
                print("Read error:", e)


if __name__ == "__main__":
    main()
