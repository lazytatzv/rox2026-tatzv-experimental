# ROX2026 CAN Communication Specification (Ultimate Edition)

通信速度: **1 Mbps**  
データフォーマット: **CAN 2.0A (Standard ID)**  
エンディアン: **リトルエンディアン (Little Endian)**  
制御周期: **10ms (100Hz)**

---

## 📥 [RDK -> STM32] 指令データ

うどんさんの提案をベースに、新たに導入した**「上下モーター別制御（バックスピン）」**に対応しつつ、8バイト以内に無駄なくパッキングした最強構成です。

### CAN ID: `0x201` (アクチュエータ・システム制御)
DLC (データ長): 8 byte

| Byte | データ名 | 型 | 説明 |
| :--- | :--- | :--- | :--- |
| `0-1` | `shooter_top_rpm` | `int16` | 上側ベルト発射モーターの目標RPM (例: 5000) |
| `2-3` | `shooter_bottom_rpm` | `int16` | 下側ベルト発射モーターの目標RPM (バックスピン用) |
| `4-5` | `dribbler_rpm` | `int16` | ドリブラー用モーターの目標RPM |
| `6` | `emergency_stop` | `uint8` | 遠隔非常停止フラグ (0: 正常, 1: 停止) |
| `7` | `reserved` | `uint8` | 予備（将来の拡張用） |

### CAN ID: `0x203` (LED制御)
DLC (データ長): 3 byte
*※0x201の8バイト枠に収まりきらないため、優先度の低いLEDは別IDとして分離。*

| Byte | データ名 | 型 | 説明 |
| :--- | :--- | :--- | :--- |
| `0` | `led_r` | `uint8` | 赤色 (0-255) |
| `1` | `led_g` | `uint8` | 緑色 (0-255) |
| `2` | `led_b` | `uint8` | 青色 (0-255) |

---

## 📤 [STM32 -> RDK] センサデータ

### CAN ID: `0x200` (スイッチ・ステータス)
DLC (データ長): 1 byte

| Byte | データ名 | 型 | 説明 |
| :--- | :--- | :--- | :--- |
| `0` | `limit_switches` | `uint8` | 各ビットにスイッチ状態を格納<br>・Bit 0: スイッチ1 (ばね発射用)<br>・Bit 1: スイッチ2<br>・Bit 2: スイッチ3 |

### CAN ID: `0x202` (IMUデータ)
DLC (データ長): 8 byte
*※Float(4byte)×4つ＝16byte は送れないため、10000倍して`int16`（2byte）に圧縮して送信する最強構成。*

| Byte | データ名 | 型 | 説明 |
| :--- | :--- | :--- | :--- |
| `0-1` | `quat_w` | `int16` | Quaternion W (実数値を10000倍) |
| `2-3` | `quat_x` | `int16` | Quaternion X (実数値を10000倍) |
| `4-5` | `quat_y` | `int16` | Quaternion Y (実数値を10000倍) |
| `6-7` | `quat_z` | `int16` | Quaternion Z (実数値を10000倍) |
