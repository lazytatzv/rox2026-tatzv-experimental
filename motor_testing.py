#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyserial>=3.5",
# ]
# ///

"""
モーター実機動作シーケンス再現プログラム (複数モーター個別連送対応版 - motor_testing.py)
================================================

【概要】
  動かしたい複数のモーターID（ID 1, 2, 3, 4）に対して、個別に高速に
  シリアル経由で速度制御指令を送信し、同時に回転させるためのテストスクリプトです。

【実行手順】
  1. USBケーブルを接続
  2. python3 motor_testing.py
"""

import serial
import time
import glob
import sys
import struct

SERIAL_BAUD = 921600
MOTOR_IDS = [1, 2, 3, 4]  # 動かしたいモーターIDのリスト
TARGET_SPEED = 5.0  # 5.0 rad/s

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

# Open serial and detect devices
CMD_OPEN_SERIAL = bytes.fromhex("41542b41540d0a")        # AT+AT\r\n
CMD_DETECT_DEV   = bytes.fromhex("41540007e84401000d0a")  # デバイス検出

def candidate_serial_ports() -> list[str]:
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in sorted(glob.glob(pattern)):
            if path not in ports:
                ports.append(path)
    return ports

def send_command(ser: serial.Serial, label: str, data: bytes) -> None:
    hex_str = " ".join(f"0x{b:02X}" for b in data)
    timestamp = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
    print(f"  [DEBUG] [{timestamp}] Send ({label}): {hex_str}")
    ser.write(data)

def main():
    ports = candidate_serial_ports()
    ser = None
    
    print("シリアルポートに接続中...")
    for port in ports:
        try:
            ser = serial.Serial(port=port, baudrate=SERIAL_BAUD, timeout=0.01)
            ser.setDTR(False)
            ser.setRTS(False)
            print(f"✓ 接続成功: {port} @ {SERIAL_BAUD}")
            break
        except Exception:
            continue

    if ser is None:
        print(f"エラー: シリアルポートに接続できませんでした。候補: {ports}", file=sys.stderr)
        sys.exit(1)

    try:
        print("\n=== 1. シリアルポート開通コマンド ===")
        send_command(ser, "Open Serial Port", CMD_OPEN_SERIAL)
        time.sleep(0.5)

        print("\n=== 2. デバイス検出コマンド ===")
        send_command(ser, "Detect Devices", CMD_DETECT_DEV)
        time.sleep(0.5)

        print(f"\n=== 3. 動作モード設定 (Velocity Mode) (IDs: {MOTOR_IDS}) ===")
        for motor_id in MOTOR_IDS:
            cmd = build_at_frame(18, motor_id, [0x05, 0x70, 0x00, 0x00, 2, 0x00, 0x00, 0x00])
            for i in range(3):
                send_command(ser, f"Set Mode ID {motor_id} #{i+1}", cmd)
                time.sleep(0.02)
        time.sleep(0.5)

        print(f"\n=== 4. モーター有効化 (Enable) (IDs: {MOTOR_IDS}) ===")
        for motor_id in MOTOR_IDS:
            cmd = build_at_frame(3, motor_id, [0, 0, 0, 0, 0, 0, 0, 0])
            for i in range(3):
                send_command(ser, f"Enable ID {motor_id} #{i+1}", cmd)
                time.sleep(0.02)
        time.sleep(0.5)

        print("   [DEBUG] 初期値 (0.0) を安全対策として送信します。")
        for motor_id in MOTOR_IDS:
            cmd = build_at_frame(18, motor_id, [0x0A, 0x70, 0x00, 0x00, 0, 0, 0, 0])
            send_command(ser, f"Init Stop ID {motor_id} (0.0)", cmd)
            time.sleep(0.02)
        time.sleep(0.1)

        print(f"\n=== 5. 速度指令送信 ({TARGET_SPEED} rad/s で 3.0秒間回転) ===")
        print("  [DEBUG] 20Hz (0.05秒間隔) で送信し続けます。")
        t_start = time.monotonic()
        count = 0
        
        # Float packing for speed control
        val_speed = struct.pack("<f", TARGET_SPEED)
        
        while time.monotonic() - t_start < 3.0:
            count += 1
            for motor_id in MOTOR_IDS:
                cmd = build_at_frame(18, motor_id, [0x0A, 0x70, 0x00, 0x00, val_speed[0], val_speed[1], val_speed[2], val_speed[3]])
                send_command(ser, f"Drive ID {motor_id} #{count}", cmd)
            time.sleep(0.05)

        print("\n=== 6. 停止フェーズ ===")
        print("  [DEBUG] 減速のため、速度 0.0 を3回送信します。")
        val_0 = struct.pack("<f", 0.0)
        for i in range(3):
            for motor_id in MOTOR_IDS:
                cmd = build_at_frame(18, motor_id, [0x0A, 0x70, 0x00, 0x00, val_0[0], val_0[1], val_0[2], val_0[3]])
                send_command(ser, f"Stop Decel ID {motor_id} #{i+1}", cmd)
                time.sleep(0.02)
            time.sleep(0.05)

        print("  [DEBUG] 完全に停止させるため、無効化 (Disable) コマンドを3回送信します。")
        for i in range(3):
            for motor_id in MOTOR_IDS:
                cmd = build_at_frame(4, motor_id, [0, 0, 0, 0, 0, 0, 0, 0])
                send_command(ser, f"Disable ID {motor_id} #{i+1}", cmd)
                time.sleep(0.02)
            time.sleep(0.05)

        print("\n✓ 正常にテスト終了しました。")

    except KeyboardInterrupt:
        print("\n\n[中断] ユーザーにより処理が中断されました。緊急停止/無効化します...")
        val_0 = struct.pack("<f", 0.0)
        # 1. 減速
        for i in range(3):
            for motor_id in MOTOR_IDS:
                cmd = build_at_frame(18, motor_id, [0x0A, 0x70, 0x00, 0x00, val_0[0], val_0[1], val_0[2], val_0[3]])
                try:
                    send_command(ser, f"Emergency Stop ID {motor_id} #{i+1}", cmd)
                except Exception:
                    pass
                time.sleep(0.02)
            time.sleep(0.05)
        # 2. 完全無効化
        for i in range(3):
            for motor_id in MOTOR_IDS:
                cmd = build_at_frame(4, motor_id, [0, 0, 0, 0, 0, 0, 0, 0])
                try:
                    send_command(ser, f"Emergency Disable ID {motor_id} #{i+1}", cmd)
                except Exception:
                    pass
                time.sleep(0.02)
            time.sleep(0.05)
    finally:
        if ser:
            ser.close()
            print("シリアルポートをクローズしました。")

if __name__ == "__main__":
    main()
