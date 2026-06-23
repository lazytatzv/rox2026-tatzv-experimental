#!/usr/bin/env python3
import serial
import time
import sys
import struct

SERIAL_BAUD = 921600

def build_at_frame(command_id: int, motor_addr: int, data: list[int]) -> bytes:
    byte0 = command_id << 3
    byte1 = 0x07
    if motor_addr == 0xFC:
        byte2 = 0xEB
        byte3 = 0xFC
    else:
        byte2 = 0xE8 | ((motor_addr >> 5) & 0x07)
        byte3 = ((motor_addr << 3) & 0xFF) | 4
    frame = bytearray([0x41, 0x54, byte0, byte1, byte2, byte3, len(data)])
    frame.extend(data)
    frame.extend([0x0D, 0x0A])
    return bytes(frame)

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 test_single_motor.py <port> <motor_id> [speed_rad_s]")
        print("Example: python3 test_single_motor.py /dev/ttyUSB1 1 2.0")
        sys.exit(1)

    port = sys.argv[1]
    motor_id = int(sys.argv[2])
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    print(f"Connecting to {port} at {SERIAL_BAUD}...")
    try:
        ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=0.05)
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    # Pack commands
    cmd_open = bytes.fromhex("41542b41540d0a")
    cmd_detect = bytes.fromhex("41540007e84401000d0a")
    cmd_set_vel_mode = build_at_frame(18, motor_id, [0x05, 0x70, 0x00, 0x00, 2, 0x00, 0x00, 0x00]) # Velocity Mode (2)
    cmd_enable = build_at_frame(3, motor_id, [0, 0, 0, 0, 0, 0, 0, 0])
    
    val_speed = struct.pack("<f", speed)
    cmd_drive = build_at_frame(18, motor_id, [0x0A, 0x70, 0x00, 0x00, val_speed[0], val_speed[1], val_speed[2], val_speed[3]])
    
    val_zero = struct.pack("<f", 0.0)
    cmd_stop = build_at_frame(18, motor_id, [0x0A, 0x70, 0x00, 0x00, val_zero[0], val_zero[1], val_zero[2], val_zero[3]])
    cmd_disable = build_at_frame(4, motor_id, [0, 0, 0, 0, 0, 0, 0, 0])

    try:
        print("Sending open serial port...")
        ser.write(cmd_open)
        time.sleep(0.1)

        print("Sending detect devices...")
        ser.write(cmd_detect)
        time.sleep(0.1)

        print(f"Setting velocity mode for Motor ID {motor_id}...")
        ser.write(cmd_set_vel_mode)
        time.sleep(0.1)

        print(f"Enabling Motor ID {motor_id}...")
        ser.write(cmd_enable)
        time.sleep(0.2)

        print(f"Driving Motor ID {motor_id} at {speed} rad/s for 3 seconds...")
        start_time = time.time()
        while time.time() - start_time < 3.0:
            ser.write(cmd_drive)
            # Read any responses to clear buffer
            if ser.in_waiting:
                ser.read(ser.in_waiting)
            time.sleep(0.05)

        print("Stopping...")
        for _ in range(3):
            ser.write(cmd_stop)
            time.sleep(0.05)

        print("Disabling...")
        for _ in range(3):
            ser.write(cmd_disable)
            time.sleep(0.05)

        print("Done!")

    except KeyboardInterrupt:
        print("Interrupted! Stopping and disabling motor...")
        for _ in range(5):
            ser.write(cmd_stop)
        time.sleep(0.1)
        for _ in range(5):
            ser.write(cmd_disable)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
