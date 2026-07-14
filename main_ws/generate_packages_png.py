import base64
import urllib.request
import ssl
import os

mermaid_code = """graph TD
    %% 1. User Input & Joystick
    subgraph "1. User Input & Joystick"
        GamePad["GamePad (Physical)"] -.->|Bluetooth / USB| JoyNode["joy::joy_node"]
        JoyNode -->|"/joy (sensor_msgs/Joy)"| TeleopNode["base_teleop::teleop"]
        JoyNode -->|"/joy"| ShooterTeleop["shooter_control::shooter_teleop"]
        
        FoxgloveHMI["Foxglove Control (HMI)"] -.->|Twist commands| TwistStampedRelay["base_teleop::twist_to_stamped"]
    end

    %% 2. Strategic Mission & Planning
    subgraph "2. Strategic Mission & Planning"
        StrategyNode["auto_strategy::strategy_node"] -->|"/cmd_vel_auto (Action)"| Nav2Stack["Navigation2 (Nav2 Stack)<br/>- bt_navigator<br/>- planner_server<br/>- controller_server<br/>- behavior_server<br/>- lifecycle_manager"]
        StrategyNode -->|"/cmd_shooter_auto (Float32)"| ShooterMux["shooter_control::shooter_mux"]
        
        TeleopNode -->|"/cmd_vel_joy (geometry_msgs/TwistStamped)"| TwistMux["twist_mux::twist_mux"]
        TeleopNode -->|"/stop_lock (std_msgs/Bool)"| TwistMux
        
        TwistStampedRelay -->|"/cmd_vel_foxglove (geometry_msgs/TwistStamped)"| TwistMux
        Nav2Stack -->|"/cmd_vel (geometry_msgs/Twist)"| TwistMux
    end

    %% 3. Perception, Vision & Localization
    subgraph "3. Perception, Vision & Localization"
        Camera["Depth Camera (Physical)"] -.->|Image stream| ImageSyncer["vision_localization::image_syncer"]
        Camera -.->|Depth Points| PointCloud2Scan["pointcloud_to_laserscan::PointCloudToLaserScanNode"]
        
        ImageSyncer -->|"/camera_synced/image_raw"| ApriltagNode["apriltag_ros::AprilTagNode"]
        ImageSyncer -->|"/camera_synced/camera_info"| ApriltagNode
        
        ApriltagNode -->|"/detections (apriltag_msgs/AprilTagDetectionArray)"| TagLocalizer["vision_localization::tag_localizer_node"]
        ApriltagNode -->|"/detections"| StrategyNode
        
        IMUDriver["libbno055_linux::bno055_perf_publisher_node"] -->|"/imu (sensor_msgs/Imu)"| EKFNode["robot_localization::ekf_filter_node"]
        IMUDriver -->|"/imu"| StabilizerNode["imu_stabilizer::imu_stabilizer_node"]
        
        TagLocalizer -->|"/apriltag_pose (PoseWithCovarianceStamped)"| EKFNode
        PointCloud2Scan -->|"/scan (sensor_msgs/LaserScan)"| Nav2Stack
        EKFNode -->|"/odometry/filtered (nav_msgs/Odometry)"| Nav2Stack
    end

    %% 4. Controller Manager & ros2_control
    subgraph "4. Controller Manager & ros2_control"
        TwistMux -->|"/cmd_vel_teleop (geometry_msgs/Twist)"| ControllerManager["controller_manager::ControllerManager"]
        StabilizerNode -->|"/yaw_rate_correction (Float64)"| ControllerManager
        
        ShooterTeleop -->|"/cmd_vel_shooter_teleop (Float32)"| ShooterMux
        ShooterMux -->|"/cmd_vel_shooter (Float32)"| ControllerManager
        
        subgraph "Loaded ros2_control Plugins"
            MecanumController["mecanum_drive_controller::MecanumDriveController"]
            JointStateBroadcaster["joint_state_broadcaster::JointStateBroadcaster"]
            RobstrideHW["robstride_driver::RobstrideHardwareInterface (Hardware Plugin)"]
            MadMotorHW["mad_motor_driver::MadMotorHardwareInterface (Hardware Plugin)"]
        end

        ControllerManager -.->|Load & Spin| MecanumController
        ControllerManager -.->|Load & Spin| JointStateBroadcaster
        ControllerManager -.->|Load & Write/Read| RobstrideHW
        ControllerManager -.->|Load & Write/Read| MadMotorHW
        
        MecanumController -->|"/joint_states"| RobotStatePublisher["robot_state_publisher::robot_state_publisher"]
        JointStateBroadcaster -->|"/joint_states"| RobotStatePublisher
    end

    %% 5. Low-Level Communications & Actuators
    subgraph "5. Low-Level Communications & Actuators"
        RobstrideHW -->|"/to_can_bus (can_msgs/Frame)"| CANSender["ros2_socketcan::socket_can_sender_node"]
        MadMotorHW -->|"/to_can_bus"| CANSender
        
        CANReceiver["ros2_socketcan::socket_can_receiver_node"] -->|"/from_can_bus (can_msgs/Frame)"| RobstrideHW
        CANReceiver -->|"/from_can_bus"| MadMotorHW
        
        CANSender -.->|"SocketCAN (can0)"| PhysicalCAN["Physical CAN Bus"]
        PhysicalCAN -.->|"SocketCAN (can0)"| CANReceiver
        
        PhysicalCAN -.->|CAN command| RobstrideMotors["Robstride Motors (Mecanum 4WD)"]
        PhysicalCAN -.->|CAN command| MADMotors["MAD Motors (Shooter/Loader)"]
    end

    %% 6. External Telemetry & HMI
    subgraph "6. External Telemetry & HMI"
        RobotStatePublisher -->|"/tf & /tf_static"| FoxgloveBridge["foxglove_bridge::foxglove_bridge"]
        EKFNode -->|"/odometry/filtered"| FoxgloveBridge
        FoxgloveBridge -.->|WebSocket| FoxgloveStudio["Foxglove Studio (HMI Visualization)"]
    end

    classDef node fill:#fff3e0,stroke:#e65100,stroke-width:1.5px;
    classDef plugin fill:#f3e5f5,stroke:#4a148c,stroke-width:1.5px;
    classDef device fill:#eceff1,stroke:#37474f,stroke-width:1px,stroke-dasharray: 5 5;
    
    class JoyNode,TeleopNode,StrategyNode,Nav2Stack,TwistMux,IMUDriver,EKFNode,StabilizerNode,TagLocalizer,ControllerManager,RobotStatePublisher,CANSender,CANReceiver,ApriltagNode,FoxgloveBridge,ImageSyncer,PointCloud2Scan,TwistStampedRelay,ShooterTeleop,ShooterMux node;
    class MecanumController,JointStateBroadcaster,RobstrideHW,MadMotorHW plugin;
    class GamePad,Camera,RobstrideMotors,MADMotors,PhysicalCAN,FoxgloveStudio,FoxgloveHMI device;
"""

# Convert to URL-safe Base64 without padding
code_bytes = mermaid_code.encode('utf-8')
base64_bytes = base64.urlsafe_b64encode(code_bytes)
base64_string = base64_bytes.decode('utf-8').rstrip('=')

# Fetch 3x scale high-resolution image with solid white background
url = f"https://mermaid.ink/img/{base64_string}?bgColor=FFFFFF&scale=3"

print(f"Requesting HIGH-RESOLUTION (3x scale) Mermaid PNG from: {url[:60]}...")
try:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    
    req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
    with urllib.request.urlopen(req, context=ctx, timeout=20) as response:
        # Determine the project root directory dynamically
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        output_path = os.path.join(project_root, "docs", "packages.png")
        
        # Ensure docs folder exists
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        with open(output_path, "wb") as f:
            f.write(response.read())
        print(f"Successfully saved HIGH-RESOLUTION Mermaid PNG to: {output_path}")
except Exception as e:
    print(f"Error occurred: {e}")
