import serial
import time

ARDUINO_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

def main():
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {ARDUINO_PORT}: {e}")
        print("Try:")
        print("  sudo chmod 666 /dev/ttyUSB0")
        print("or permanent:")
        print("  sudo usermod -aG dialout $USER")
        print("  sudo reboot")
        return

    time.sleep(2)

    print("Connected to Arduino motor controller.")
    print("Commands:")
    print("  F speed duration_ms")
    print("  B speed duration_ms")
    print("  L speed duration_ms")
    print("  R speed duration_ms")
    print("  P speed duration_ms")
    print("  S")
    print()
    print("Examples:")
    print("  F 120 2000")
    print("  P 140 3000")
    print("  S")
    print("  q")

    while True:
        cmd = input("motor> ").strip()

        if not cmd:
            continue

        if cmd.lower() in ["q", "quit", "exit"]:
            ser.write(b"S\n")
            print("Sent stop command. Exiting.")
            break

        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()

        time.sleep(0.2)

        while ser.in_waiting:
            response = ser.readline().decode("utf-8", errors="ignore").strip()
            if response:
                print("Arduino:", response)

    ser.close()

if __name__ == "__main__":
    main()
