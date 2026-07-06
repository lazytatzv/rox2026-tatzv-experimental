import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from enum import Enum, auto

# 状態遷移用のEnum（Udonさんのレビュー要件に準拠！）
class SpringState(Enum):
    INIT = auto()           # 初期状態（スイッチの状態確認）
    RELOADING = auto()      # 装填中（スイッチがONになるまで回す）
    READY = auto()          # 発射準備完了（待機）
    FIRING_ESCAPE = auto()  # 発射開始（スイッチがOFFになるまで回す）

class SpringController(Node):
    def __init__(self):
        super().__init__('spring_controller')

        # パラメータの宣言（マジックナンバー排除）
        self.declare_parameter('reload_speed', 20.0)      # 装填速度 (rad/s)
        self.declare_parameter('fire_speed', 20.0)        # 発射速度 (rad/s)
        self.declare_parameter('joy_l2_index', 6)         # L2のインデックス（配列の6番目）
        self.declare_parameter('joy_r1_index', 5)         # R1のインデックス（配列の5番目）
        self.declare_parameter('control_hz', 100.0)       # 制御周期 (Hz)
        self.declare_parameter('topic_joy', '/joy')
        self.declare_parameter('topic_limit_switch', '/shooter/limit_switch')
        self.declare_parameter('topic_speed_out', '/edulite_speed')

        self.reload_speed = self.get_parameter('reload_speed').value
        self.fire_speed = self.get_parameter('fire_speed').value
        self.idx_l2 = self.get_parameter('joy_l2_index').value
        self.idx_r1 = self.get_parameter('joy_r1_index').value
        control_hz = self.get_parameter('control_hz').value
        t_joy = self.get_parameter('topic_joy').value
        t_limit = self.get_parameter('topic_limit_switch').value
        t_speed = self.get_parameter('topic_speed_out').value

        # 状態変数
        self.state = SpringState.INIT
        self.is_limit_sw_on = False
        self.is_trigger_pressed = False

        # Subscribers
        self.sub_joy = self.create_subscription(
            Joy, t_joy, self.joy_callback, 10)
            
        # 先ほど実装したばかりの完璧なリミットスイッチトピックを使います
        self.sub_limit = self.create_subscription(
            Bool, t_limit, self.limit_callback, 10)

        # Publisher
        self.pub_speed = self.create_publisher(Float32, t_speed, 10)

        # Timer (100Hzの制御ループ)
        self.timer = self.create_timer(1.0 / control_hz, self.control_loop)

        self.get_logger().info("Spring Controller started. State: INIT")

    def joy_callback(self, msg: Joy):
        # L2とR1が両方押されているか判定 (安全装置付きトリガー)
        # ※Udonさんの指示に従い、配列のインデックスを使用。
        # L2が軸(axes)かボタン(buttons)かはコントローラ環境によりますが、指示通り配列要素で判定します。
        
        try:
            # 一般的なJoy設定だとL2はaxes、R1はbuttonsになることが多いですが、
            # 両方buttons配列に入っていると仮定して安全に処理します。
            # もし環境によってエラーが出るならここを調整します。
            l2_pressed = (msg.buttons[self.idx_l2] == 1) if len(msg.buttons) > self.idx_l2 else False
            r1_pressed = (msg.buttons[self.idx_r1] == 1) if len(msg.buttons) > self.idx_r1 else False
            
            # axesとしてアサインされている環境向けのフォールバック（L2が押し込まれているか）
            if not l2_pressed and len(msg.axes) > self.idx_l2:
                # 軸の値が負(押し込まれている)なら押されたと判定
                if msg.axes[self.idx_l2] < -0.5:
                    l2_pressed = True

            self.is_trigger_pressed = l2_pressed and r1_pressed
        except IndexError:
            self.get_logger().error("Joy message index out of range!")

    def limit_callback(self, msg: Bool):
        # リミットスイッチの状態を更新（1ならON, 0ならOFF）
        self.is_limit_sw_on = msg.data

    def control_loop(self):
        target_speed = 0.0

        # --- 完璧なステートマシン（状態遷移） ---
        if self.state == SpringState.INIT:
            if self.is_limit_sw_on:
                self.state = SpringState.READY
                self.get_logger().info("Loaded -> READY")
            else:
                self.state = SpringState.RELOADING
                self.get_logger().info("Not Loaded -> RELOADING")

        elif self.state == SpringState.RELOADING:
            target_speed = self.reload_speed
            # リミットスイッチが反応するまで回す
            if self.is_limit_sw_on:
                self.state = SpringState.READY
                self.get_logger().info("Finished Reloading -> READY")

        elif self.state == SpringState.READY:
            target_speed = 0.0
            # トリガーが引かれたら発射開始
            if self.is_trigger_pressed:
                self.state = SpringState.FIRING_ESCAPE
                self.get_logger().info("FIRE! -> FIRING_ESCAPE")

        elif self.state == SpringState.FIRING_ESCAPE:
            target_speed = self.fire_speed
            # 時間ではなく、リミットスイッチがOFFになる（カムが外れる）まで回す！
            if not self.is_limit_sw_on:
                self.state = SpringState.RELOADING
                self.get_logger().info("Escaped limit switch -> RELOADING")

        # モーターへ速度を送信
        out_msg = Float32()
        out_msg.data = float(target_speed)
        self.pub_speed.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
