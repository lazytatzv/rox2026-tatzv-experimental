#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyserial>=3.5",
# ]
# ///

"""
RobStride Motor ID Configuration Script (Official Communication Type 7)
====================================================================
Usage:
  uv run motor_testing/set_motor_id.py --current <current_id> --new <new_id> [--port /dev/ttyUSB0]
"""

import serial
import time
import sys
import argparse

SERIAL_BAUD = 921600

def build_at_frame_raw(can_id: int, data: list[int]) -> bytes:
    # Shift CAN ID left by 3, and set bit 2 (value 4) for Extended Frame
    shifted_id = (can_id << 3) | 4
    
    byte0 = (shifted_id >> 24) & 0xFF
    byte1 = (shifted_id >> 16) & 0xFF
    byte2 = (shifted_id >> 8) & 0xFF
    byte3 = shifted_id & 0xFF
    
    frame = bytearray([0x41, 0x54, byte0, byte1, byte2, byte3, len(data)])
    frame.extend(data)
    frame.extend([0x0D, 0x0A])
    return bytes(frame)

def main():
    parser = argparse.ArgumentParser(description="RobStride Motor ID Setup Tool (Type 7)")
    parser.add_argument("--current", type=int, required=True, help="Current Motor ID (e.g., 5)")
    parser.add_argument("--new", type=int, required=True, help="New Motor ID to set (e.g., 1)")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port path")
    args = parser.parse_args()

    current_id = args.current
    new_id = args.new

    try:
        ser = serial.Serial(port=args.port, baudrate=SERIAL_BAUD, timeout=0.1)
        ser.setDTR(False)
        ser.setRTS(False)
        print(f"Connected: {args.port} @ {SERIAL_BAUD}")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

    try:
        # 1. Open serial link
        ser.write(bytes.fromhex("41542b41540d0a"))
        time.sleep(0.5)

        # 2. Build and send Type 7 CAN frames
        print(f"Changing ID from {current_id} -> {new_id} using Communication Type 7...")

        # Form 1: CAN ID has new_id and current_id, payload is empty/zeros
        # 29-bit CAN ID: (7 << 24) | (new_id << 16) | (current_id << 8) | current_id
        can_id_1 = (7 << 24) | (new_id << 16) | (current_id << 8) | current_id
        cmd_1 = build_at_frame_raw(can_id_1, [0] * 8)
        print(f"Sending Command: {' '.join(f'0x{b:02X}' for b in cmd_1)}")
        ser.write(cmd_1)
        time.sleep(0.1)

        # Form 2: CAN ID has new_id and current_id, payload has new_id in data[0]
        cmd_2 = build_at_frame_raw(can_id_1, [new_id] + [0] * 7)
        ser.write(cmd_2)
        time.sleep(0.1)

        # Form 3: CAN ID uses broadcast (0x7F or 0xFC) as target, payload has new_id
        # 29-bit CAN ID: (7 << 24) | (new_id << 16) | (current_id << 8) | 0x7F
        can_id_3 = (7 << 24) | (new_id << 16) | (current_id << 8) | 0x7F
        cmd_3 = build_at_frame_raw(can_id_3, [new_id] + [0] * 7)
        ser.write(cmd_3)
        time.sleep(0.1)

        # Form 4: Target is current ID, new ID is in payload
        # 29-bit CAN ID: (7 << 24) | (current_id << 8) | current_id
        can_id_4 = (7 << 24) | (current_id << 8) | current_id
        cmd_4 = build_at_frame_raw(can_id_4, [new_id] + [0] * 7)
        ser.write(cmd_4)
        time.sleep(0.1)

        print(f"\n✓ ID change commands sent successfully!")
        print("Please restart the motor if needed (though it should take effect immediately).")

    except KeyboardInterrupt:
        print("\nAborted by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
