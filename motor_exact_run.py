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

SERIAL_BAUD = 921600

# ユーザーから提供された生の送信バイトデータ (16進数文字列から変換)
CMD_OPEN_SERIAL  = bytes.fromhex("41542b41540d0a")        # AT+AT\r\n (シリアルポート開通)
CMD_DETECT_DEV   = bytes.fromhex("41540007e84401000d0a")  # デバイス検出
CMD_SET_VEL_MODE = bytes.fromhex("41549007ebfc0805700000020000000d0a") # Velocity Mode設定
CMD_ENABLE       = bytes.fromhex("41541807ebfc0800000000000000000d0a") # Enable

# 速度 45.0 rad/s で回転させるコマンド (データ部末尾: 00 00 34 42 -> Float 45.0)
CMD_DRIVE_45RAD  = bytes.fromhex("41549007ebfc080a700000000034420d0a")

# 速度 0.0 rad/s で停止させるコマンド (データ部末尾: 00 00 00 00 -> Float 0.0)
CMD_STOP         = bytes.fromhex("41549007ebfc080a700000000000000d0a")

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
        print("  [DEBUG] 安全に停止させるため、速度 0.0 を5回送信します。")
        for i in range(5):
            send_command(ser, f"Stop #{i+1}", CMD_STOP)
            time.sleep(0.05)

        print("\n✓ 正常にテスト終了しました。")

    except KeyboardInterrupt:
        print("\n\n[中断] ユーザーにより処理が中断されました。緊急停止します...")
        for i in range(5):
            try:
                send_command(ser, f"Emergency Stop #{i+1}", CMD_STOP)
            except Exception:
                pass
            time.sleep(0.05)
    finally:
        if ser:
            ser.close()
            print("シリアルポートをクローズしました。")

if __name__ == "__main__":
    main()
