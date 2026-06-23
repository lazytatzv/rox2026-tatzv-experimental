#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyserial>=3.5",
# ]
# ///

"""
RobStride Motor ID Configuration Script (Native CAN-to-USB)
==========================================================
Usage:
  uv run set_motor_id.py --id <new_id> [--port /dev/ttyUSB0]
"""

import serial
import time
import glob
import sys
import argparse

SERIAL_BAUD = 921600

def candidate_serial_ports() -> list[str]:
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in sorted(glob.glob(pattern)):
            if path not in ports:
                ports.append(path)
    return ports

def build_id_set_cmd(new_id: int) -> bytes:
    # Command 4 (Set CAN ID) -> 4 << 3 = 32 = 0x20
    # Target ID is encoded as (new_id << 3) | 4 as verified by real packet logs
    id_byte = (new_id << 3) | 4
    return bytes([
        0x41, 0x54,
        0x20, 0x07, 0xE8, id_byte,
        0x08,
        0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0D, 0x0A
    ])

def decode_motor_id_from_frame(frame: bytes) -> int | None:
    if len(frame) < 17:
        return None
    if frame[0] != 0x41 or frame[1] != 0x54: # 'AT'
        return None
    
    # data[4] (Byte 2) and data[5] (Byte 3) contain encoded motor_id
    byte2 = frame[4]
    byte3 = frame[5]
    
    # Inverse of: byte2 = 0xE8 | (id >> 5), byte3 = (id << 3) | 4
    motor_id = ((byte2 & 0x07) << 5) | ((byte3 & 0xF8) >> 3)
    return motor_id

def main():
    parser = argparse.ArgumentParser(description="RobStride Motor ID Setup Tool (Native CAN-to-USB)")
    parser.add_argument("--id", type=int, required=True, help="New Motor ID to set (e.g., 1 or 2)")
    parser.add_argument("--port", type=str, default=None, help="Serial port path (e.g., /dev/ttyUSB0)")
    args = parser.parse_args()

    new_id = args.id
    if not (1 <= new_id <= 127):
        print(f"Error: Motor ID must be between 1 and 127. Given: {new_id}", file=sys.stderr)
        sys.exit(1)

    ports = [args.port] if args.port else candidate_serial_ports()
    ser = None
    
    print("Connecting to serial port...")
    for port in ports:
        try:
            ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=0.01)
            ser.setDTR(False)
            ser.setRTS(False)
            print(f"✓ Connected: {port} @ {SERIAL_BAUD}")
            break
        except Exception:
            continue

    if ser is None:
        print(f"Error: Could not connect to serial port. Checked: {ports}", file=sys.stderr)
        sys.exit(1)

    try:
        # 1. Open serial port link
        CMD_OPEN_SERIAL = bytes.fromhex("41542b41540d0a")
        print("\n1. Opening serial link...")
        ser.write(CMD_OPEN_SERIAL)
        time.sleep(0.5)

        # 2. Detect devices
        CMD_DETECT_DEV = bytes.fromhex("41540007e84401000d0a")
        print("2. Detecting connected devices...")
        ser.write(CMD_DETECT_DEV)
        time.sleep(0.5)

        # 3. Send ID setting command
        cmd = build_id_set_cmd(new_id)
        hex_str = " ".join(f"0x{b:02X}" for b in cmd)
        print(f"\n3. Sending ID change command (New ID: {new_id})...")
        print(f"   Frame: {hex_str}")
        
        # Send 3 times for robustness
        for i in range(3):
            ser.write(cmd)
            time.sleep(0.05)

        print("\n✓ Command sent successfully!")
        print("==================================================")
        print("IMPORTANT: Please power cycle (turn OFF and ON)")
        print("the motor power supply to apply the new ID permanently!")
        print("==================================================")
        
        # 4. Verification phase
        print("\n4. Checking motor feedback to verify ID...")
        print("   (Listening on port for 3 seconds. Please ensure the motor is powered ON)")
        
        ser.reset_input_buffer()
        start_time = time.monotonic()
        rx_buffer = bytearray()
        detected_ids = set()
        
        while time.monotonic() - start_time < 3.0:
            if ser.in_waiting > 0:
                rx_buffer.extend(ser.read(ser.in_waiting))
                
                while len(rx_buffer) >= 17:
                    idx = rx_buffer.find(b'AT')
                    if idx == -1:
                        if rx_buffer[-1:] == b'A':
                            rx_buffer = rx_buffer[-1:]
                        else:
                            rx_buffer.clear()
                        break
                    
                    if idx > 0:
                        del rx_buffer[:idx]
                        
                    if len(rx_buffer) < 17:
                        break
                        
                    if rx_buffer[15] == 0x0D and rx_buffer[16] == 0x0A:
                        frame = bytes(rx_buffer[:17])
                        del rx_buffer[:17]
                        
                        motor_id = decode_motor_id_from_frame(frame)
                        if motor_id is not None:
                            detected_ids.add(motor_id)
                    else:
                        del rx_buffer[0]
            time.sleep(0.01)
            
        if detected_ids:
            print(f"\n✓ Detected Active Motor ID(s) on bus: {list(detected_ids)}")
            if new_id in detected_ids:
                print(f"  --> CONFIRMED: Motor ID {new_id} is active and communicating!")
            else:
                print(f"  --> WARNING: Target ID {new_id} not found in feedback. Active IDs: {list(detected_ids)}")
                print("      Did you power cycle the motor? ID changes apply only after a restart.")
        else:
            print("\n  --> No motor feedback detected. (Power cycle the motor and ensure connections are tight)")

    except KeyboardInterrupt:
        print("\nAborted by user.")
    finally:
        if ser:
            ser.close()
            print("\nSerial port closed.")

if __name__ == "__main__":
    main()
