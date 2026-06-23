#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyserial>=3.5",
# ]
# ///

"""
モーター実機動作シーケンス再現プログラム
================================================

【概要】
  公式GUIツールとシリアルモニターで確認された「実機が動作する生のバイトデータ」を
  完全に同じ手順・同じタイミングで送信し、モーターを確実に回転させるためのテストスクリプトです。

【実行手順】
  1. USBケーブルをPC（RDK X5）に接続
  2. uv run motor_testing/motor_exact_run.py
"""

import serial
import time
import glob
import os
import sys
import struct

SERIAL_BAUD = 921600
MOTOR_ID = 1  # Target Motor ID (Automatic ID mask generation)

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
CMD_OPEN_SERIAL  = bytes.fromhex("41542b41540d0a")        # AT+AT\r\n (シリアルポート開通)
CMD_DETECT_DEV   = bytes.fromhex("41540007e84401000d0a")  # デバイス検出

# Dynamically generated command payloads
CMD_SET_VEL_MODE = build_at_frame(18, MOTOR_ID, [0x05, 0x70, 0x00, 0x00, 2, 0x00, 0x00, 0x00]) # Velocity Mode (2)
CMD_ENABLE       = build_at_frame(3, MOTOR_ID, [0, 0, 0, 0, 0, 0, 0, 0]) # Enable

# Float packing for speed control
val_45 = struct.pack("<f", 45.0)
CMD_DRIVE_45RAD  = build_at_frame(18, MOTOR_ID, [0x0A, 0x70, 0x00, 0x00, val_45[0], val_45[1], val_45[2], val_45[3]])

val_0 = struct.pack("<f", 0.0)
CMD_STOP         = build_at_frame(18, MOTOR_ID, [0x0A, 0x70, 0x00, 0x00, val_0[0], val_0[1], val_0[2], val_0[3]])

# ============================================================
# シリアルポート候補のリストを取得
# ============================================================

def candidate_serial_ports() -> list[str]:
    """接続候補となるシリアルポートの一覧を返す"""
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in sorted(glob.glob(pattern)):
            if path not in ports:
                ports.append(path)
    return ports

# ============================================================
# デバッグ出力付きシリアル送信関数
# ============================================================

def send_command(ser: serial.Serial, label: str, data: bytes) -> None:
    """データをシリアル送信し、その生のバイナリデータとタイムスタンプを出力する"""
    hex_str = " ".join(f"0x{b:02X}" for b in data)
    timestamp = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
    print(f"  [DEBUG] [{timestamp}] Send ({label}): {hex_str}")
    ser.write(data)

# ============================================================
# メイン処理
# ============================================================

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
        print("  [DEBUG] 送信データ: AT+AT\\r\\n")
        send_command(ser, "Open Serial Port", CMD_OPEN_SERIAL)
        time.sleep(0.5)

        print("\n=== 2. デバイス検出コマンド ===")
        send_command(ser, "Detect Devices", CMD_DETECT_DEV)
        time.sleep(0.5)

        print("\n=== 3. 速度制御モード (Velocity Mode) 設定 ===")
        print("  [DEBUG] 動作モード -> Speed Mode (Mode = 2)")
        send_command(ser, "Set Velocity Mode", CMD_SET_VEL_MODE)
        time.sleep(0.5)

        print("\n=== 4. モーター有効化 (Enable) ===")
        send_command(ser, "Enable", CMD_ENABLE)
        time.sleep(0.5)

        print("\n=== 5. 速度指令送信 (45.0 rad/s で 3秒間回転) ===")
        print("  [DEBUG] 20Hz (0.05秒間隔) で送信し続けます。")
        t_start = time.monotonic()
        count = 0
        while time.monotonic() - t_start < 3.0:
            count += 1
            send_command(ser, f"Drive #{count}", CMD_DRIVE_45RAD)
            time.sleep(0.05)

        print("\n=== 6. 停止フェーズ ===")
        print("  [DEBUG] 減速のため、速度 0.0 を3回送信します。")
        for i in range(3):
            send_command(ser, f"Stop Decel #{i+1}", CMD_STOP)
            time.sleep(0.02)
            time.sleep(0.05)

        print("  [DEBUG] 完全に停止させるため、無効化 (Disable) コマンドを3回送信します。")
        disable_cmd = build_at_frame(4, MOTOR_ID, [0, 0, 0, 0, 0, 0, 0, 0])
        for i in range(3):
            send_command(ser, f"Disable #{i+1}", disable_cmd)
            time.sleep(0.02)
            time.sleep(0.05)

        print("\n✓ 正常にテスト終了しました。")

    except KeyboardInterrupt:
        print("\n\n[中断] ユーザーにより処理が中断されました。緊急停止/無効化します...")
        # 1. 減速
        for i in range(3):
            try:
                send_command(ser, f"Emergency Stop #{i+1}", CMD_STOP)
            except Exception:
                pass
            time.sleep(0.02)
            time.sleep(0.05)
        # 2. 完全無効化
        disable_cmd = build_at_frame(4, MOTOR_ID, [0, 0, 0, 0, 0, 0, 0, 0])
        for i in range(3):
            try:
                send_command(ser, f"Emergency Disable #{i+1}", disable_cmd)
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
