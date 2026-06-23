import serial
import time

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

def parse_sensor_line(line: str):
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
        return

    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected.")
    print("Reading STM32 distance sensor data...")
    print("Expected format: front,left,right,back,error")
    print("-" * 60)

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

            print(
                f"Front: {front:3d} cm | "
                f"Left: {left:3d} cm | "
                f"Right: {right:3d} cm | "
                f"Back: {back:3d} cm | "
                f"Error: {error}"
            )

        except KeyboardInterrupt:
            print("\nStopped.")
            break

        except Exception as e:
            print("Read error:", e)


if __name__ == "__main__":
    main()
