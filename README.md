# ROX2026 Tatzv Experimental

The ultimate high-performance mecanum robot controller. Optimized for reliability, speed, and developer experience.

## 🚀 Key Features
- **All-C++ Zero-Copy Architecture**: High-speed communication using ROS 2 Composable Components.
- **Safety First**: Hardware-level Watchdog, DualSense Emergency Stop, and Lifecycle management.
- **Modern Stack**: Nix Flakes for tools, Docker for runtime, and Foxglove for visualization.
- **Self-Healing CI**: Automated style fixing and cross-distribution (Humble/Jazzy) validation.

## 🏗️ System Architecture
```mermaid
graph TD
    subgraph "Input & UI"
        DualSense[DualSense Controller] --> joy_node
        joy_node --> base_teleop[base_teleop: ARM/STOP Logic]
        base_teleop --> twist_mux
    end

    subgraph "Logic & Calculation (HAL)"
        twist_mux --> kinematics[mecanum_kinematics: Watchdog & Odom]
        kinematics --> dispatcher[speed_dispatcher]
    end

    subgraph "Actuator Layer (Zero-Copy)"
        dispatcher -- rad/s --> robstride[robstride_at/can_driver]
        robstride -- SerialFrame/CanFrame --> gateway[bus_gateway]
        gateway -- Physical --> Motors((EL05 Motors))
    end

    subgraph "Monitoring"
        kinematics -- TF/Odom --> Foxglove[Foxglove Studio]
        robstride -- JointState --> Foxglove
    end
```

## 🛠️ Quick Start
1. **Prepare Host**: Install Nix and `direnv`.
2. **Setup**: `direnv allow` (This installs all tools and sets up X11 automatically).
3. **Build**: `make build` then `make colcon`.
4. **Launch**: `make launch` (or `make virtual` for mock testing).

## 🎮 Controls (DualSense)
- **Select (Create)**: ARM system (Activate movement).
- **Touchpad Click**: STOP system (Universal lock).
- **L Stick / R Stick**: Strafe and Yaw movement.
- **R1 / L1**: High-precision deadband control.
```
