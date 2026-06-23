#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "pyserial>=3.5",
#     "pyyaml>=6.0.1",
# ]
# ///

"""
モーター実機動作シーケンス再現プログラム (複数モーター個別連送対応版)
================================================

【概要】
  config.yaml からパラメータを読み込み、公式GUIツールと同様のシーケンスでモーター制御を行います。
  複数モーターID（例：[1, 2, 3, 4]）を指定した場合、各モーターへ個別に高速連送を行います。
  位置制御 (position) と速度制御 (velocity) を切り替えてテスト可能です。
    1. シリアル開通 (AT+AT\r\n)
    2. デバイス検出
    3. 指定された動作モード (Position = 1, Velocity = 2) に設定
    4. Enable (有効化)
    5. 目標パラメータ (位置 rad / 速度 rad/s) の連送 (駆動)
    6. Stop (位置 0.0 / 速度 0.0)

【実行手順】
  1. USBケーブルをPC（RDK X5）に接続
  2. uv run motor_testing/motor_testing.py
"""

import serial
import time
import glob
import os
import sys
import struct
import yaml

# 設定ファイルのパス (config.yaml を読み込む)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(SCRIPT_DIR, "config.yaml")

# ============================================================
# 設定読み込み関数
# ============================================================

def load_config(config_path: str) -> dict:
    """YAMLファイルから設定を読み込む。存在しない場合はデフォルト値を返す"""
    default_config = {
        "motor": {
            "ids": [1, 2, 3, 4],
            "mode": "position",
            "speed": 10.0,
            "position": 0.0,
            "limit_current": 5.0,
            "duration": 3.0
        },
        "serial": {
            "port": None,
            "baudrate": 921600
        }
    }

    if not os.path.exists(config_path):
        print(f"[DEBUG] 設定ファイル {config_path} が見つからないため、デフォルト値を使用します。")
        return default_config

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            user_config = yaml.safe_load(f)
            if user_config:
                if "motor" in user_config:
                    default_config["motor"].update(user_config["motor"])
                if "serial" in user_config:
                    default_config["serial"].update(user_config["serial"])
        
        # 単一idまたはidsリストの両方を許容する
        motor_settings = default_config["motor"]
        if "ids" not in motor_settings and "id" in motor_settings:
            val = motor_settings["id"]
            motor_settings["ids"] = [val] if not isinstance(val, list) else val

        # idsの中身を数値化
        parsed_ids = []
        for val in motor_settings["ids"]:
            if isinstance(val, str):
                if val.lower().startswith("0x"):
                    parsed_ids.append(int(val, 16))
                else:
                    parsed_ids.append(int(val))
            else:
                parsed_ids.append(int(val))
        motor_settings["ids"] = parsed_ids
                
        print(f"[DEBUG] 設定ファイルを読み込みました: {config_path}")
        return default_config
    except Exception as e:
        print(f"[WARN] 設定ファイルの読み込みに失敗しました。デフォルト設定を使用します: {e}", file=sys.stderr)
        return default_config

# ============================================================
# 生データ定義および動的コマンド生成関数
# ============================================================

CMD_OPEN_SERIAL  = bytes.fromhex("41542b41540d0a")        # AT+AT\r\n (シリアルポート開通)
CMD_DETECT_DEV   = bytes.fromhex("41540007e84401000d0a")  # デバイス検出

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

def build_mode_select_cmd(motor_addr: int, mode_name: str) -> bytes:
    """動作モードを設定するためのATフレームを生成 (Position=1, Velocity=2)"""
    mode_value = 1 if mode_name.lower() == "position" else 2
    return build_at_frame(18, motor_addr, [0x05, 0x70, 0x00, 0x00, mode_value, 0x00, 0x00, 0x00])

def build_limit_spd_cmd(motor_addr: int, limit_spd: float) -> bytes:
    """位置制御時の制限速度 (rad/s) を設定するためのATフレームを生成 (レジスタ 0x7017)"""
    val_bytes = struct.pack("<f", float(limit_spd))
    return build_at_frame(18, motor_addr, [0x17, 0x70, 0x00, 0x00, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]])

def build_limit_cur_cmd(motor_addr: int, limit_cur: float) -> bytes:
    """制限電流 (A) を設定するためのATフレームを生成 (レジスタ 0x7018)"""
    val_bytes = struct.pack("<f", float(limit_cur))
    return build_at_frame(18, motor_addr, [0x18, 0x70, 0x00, 0x00, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]])

def build_enable_cmd(motor_addr: int) -> bytes:
    """モーターをイネーブル（有効化）にするためのATフレームを生成"""
    return build_at_frame(3, motor_addr, [0, 0, 0, 0, 0, 0, 0, 0])

def build_velocity_cmd(motor_addr: int, target_speed: float) -> bytes:
    """指定した実速度 (rad/s) で回転させるための速度指令ATフレームを生成 (レジスタ 0x700A)"""
    val_bytes = struct.pack("<f", float(target_speed))
    return build_at_frame(18, motor_addr, [0x0A, 0x70, 0x00, 0x00, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]])

def build_position_cmd(motor_addr: int, target_position: float) -> bytes:
    """指定した目標位置 (rad) に制御するための位置指令ATフレームを生成 (レジスタ 0x7016)"""
    val_bytes = struct.pack("<f", float(target_position))
    return build_at_frame(18, motor_addr, [0x16, 0x70, 0x00, 0x00, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]])

# ============================================================
# シリアルポート候補のリストを取得
# ============================================================

def candidate_serial_ports(specified_port: str | None) -> list[str]:
    """接続候補となるシリアルポートの一覧を返す"""
    ports = []
    if specified_port:
        ports.append(specified_port)
    else:
        ports.extend(["/dev/ttyUSB0", "/dev/ttyUSB1"])
        
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
    print("=" * 70)
    print("  モーター実機動作プログラム (複数モーター個別制御版) 起動中...")
    print("=" * 70)

    # 1. 設定のロード
    config = load_config(CONFIG_FILE)
    
    motor_settings = config["motor"]
    serial_settings = config["serial"]
    
    motor_ids = motor_settings["ids"]
    control_mode = motor_settings.get("mode", "position").lower()
    target_speed = motor_settings["speed"]
    target_position = motor_settings["position"]
    limit_current = motor_settings.get("limit_current", 5.0)
    duration = motor_settings["duration"]
    
    baudrate = serial_settings["baudrate"]
    specified_port = serial_settings["port"]

    print("[DEBUG] 読み込んだ設定パラメータ:")
    print(f"  - モーター IDs   : {motor_ids}")
    print(f"  - 制御モード     : {control_mode.upper()}")
    if control_mode == "position":
        print(f"  - 目標位置 (rad) : {target_position} (約 {target_position * 57.2958:.1f} 度)")
        print(f"  - 制限速度 (rad/s): {target_speed}")
        print(f"  - 制限電流 (A)   : {limit_current}")
    else:
        print(f"  - 目標速度 (rad/s): {target_speed}")
    print(f"  - 動作時間       : {duration} 秒")
    print(f"  - 指定ポート     : {specified_port or '自動検出'}")
    print(f"  - ボーレート     : {baudrate}")
    print("-" * 70)

    # 2. シリアルポートへの接続
    ports = candidate_serial_ports(specified_port)
    ser = None
    
    print("シリアルポートに接続中...")
    for port in ports:
        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.01)
            ser.setDTR(False)
            ser.setRTS(False)
            print(f"✓ 接続成功: {port} @ {baudrate}")
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

        print(f"\n=== 3. 動作モード設定 ({control_mode.upper()} Mode) ===")
        for motor_id in motor_ids:
            for i in range(3):
                send_command(ser, f"Set Mode ID {motor_id} #{i+1}", build_mode_select_cmd(motor_id, control_mode))
                time.sleep(0.02)
        time.sleep(0.5)

        if control_mode == "position":
            print("\n=== 3.5. 制限速度・制限電流設定 ===")
            for motor_id in motor_ids:
                for i in range(3):
                    send_command(ser, f"Set Limit Speed ID {motor_id} #{i+1}", build_limit_spd_cmd(motor_id, target_speed))
                    time.sleep(0.02)
                time.sleep(0.05)
                for i in range(3):
                    send_command(ser, f"Set Limit Current ID {motor_id} #{i+1}", build_limit_cur_cmd(motor_id, limit_current))
                    time.sleep(0.02)
            time.sleep(0.5)

        print("\n=== 4. モーター有効化 (Enable) ===")
        for motor_id in motor_ids:
            for i in range(3):
                send_command(ser, f"Enable ID {motor_id} #{i+1}", build_enable_cmd(motor_id))
                time.sleep(0.02)
        time.sleep(0.5)

        # 初期値の送信
        print("   [DEBUG] 初期値 (0.0) を安全対策として送信します。")
        for motor_id in motor_ids:
            if control_mode == "position":
                send_command(ser, f"Init Pos ID {motor_id} (0.0)", build_position_cmd(motor_id, 0.0))
            else:
                send_command(ser, f"Init Stop ID {motor_id} (0.0)", build_velocity_cmd(motor_id, 0.0))
            time.sleep(0.02)
        time.sleep(0.1)

        # 駆動フェーズ
        print(f"\n=== 5. 速度/位置指令送信 ({duration}秒間) ===")
        print("  [DEBUG] 20Hz (0.05秒間隔) で送信し続けます。")
        t_start = time.monotonic()
        count = 0
        while time.monotonic() - t_start < duration:
            count += 1
            for motor_id in motor_ids:
                if control_mode == "position":
                    cmd = build_position_cmd(motor_id, target_position)
                else:
                    cmd = build_velocity_cmd(motor_id, target_speed)
                send_command(ser, f"Drive ID {motor_id} #{count}", cmd)
            time.sleep(0.05)

        print("\n=== 6. 停止・原点復帰フェーズ ===")
        print("  [DEBUG] モーターを安全に初期位置（0.0）に戻すコマンドを5回送信します。")
        for i in range(5):
            for motor_id in motor_ids:
                if control_mode == "position":
                    stop_cmd = build_position_cmd(motor_id, 0.0)
                else:
                    stop_cmd = build_velocity_cmd(motor_id, 0.0)
                send_command(ser, f"Stop ID {motor_id} #{i+1}", stop_cmd)
            time.sleep(0.05)

        print("\n✓ 正常にテスト終了しました。")

    except KeyboardInterrupt:
        print("\n\n[中断] ユーザーにより処理が中断されました。原点復帰/緊急停止します...")
        for i in range(5):
            for motor_id in motor_ids:
                if control_mode == "position":
                    stop_cmd = build_position_cmd(motor_id, 0.0)
                else:
                    stop_cmd = build_velocity_cmd(motor_id, 0.0)
                try:
                    send_command(ser, f"Emergency Stop ID {motor_id} #{i+1}", stop_cmd)
                except Exception:
                    pass
            time.sleep(0.05)
    finally:
        if ser:
            ser.close()
            print("シリアルポートをクローズしました。")

if __name__ == "__main__":
    main()
