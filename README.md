# ROS2 Robotic Arm 7-DOF

Educational project: a complete ROS2 Humble control stack for a 7-DOF robotic arm (6 joints + gripper) driven by CubeMars motors over CAN.

## Tech stack

- **ROS2 Humble**
- **C++17** — hardware interface, CAN communication, RViz panel
- **Python** — launch files, direct serial trajectory tool
- **MoveIt2** — motion planning
- **ros2_control** — hardware abstraction layer

## Project structure

```
ros2_robotic_arm/
├── src/
│   ├── arm6dof/
│   │   ├── src/arm/
│   │   │   ├── ArmController.hpp/cpp         # Motor control: MIT protocol, per-motor params
│   │   │   ├── CanBridge.hpp/cpp             # Waveshare USB-CAN-A serial driver (std + ext frames)
│   │   │   ├── ArmHardwareInterface.hpp/cpp  # ros2_control SystemInterface plugin
│   │   │   └── armnode.cpp                  # Stub node (publishers not yet implemented)
│   │   ├── src/rviz/
│   │   │   └── ArmControlPanel.hpp/cpp       # RViz Qt panel: enable/disable/zero per motor
│   │   ├── arm6dof_plugin.xml               # ros2_control plugin registration
│   │   ├── arm_panel_plugin.xml             # RViz panel plugin registration
│   │   ├── launch/main.launch.py
│   │   └── urdf/arm6dof.urdf
│   └── arm6dof_moveit_config/
│       ├── config/                           # Kinematics, planners, controllers
│       └── launch/
└── tools/
    └── trajectory.py                         # Direct serial MIT tool (no ROS2)
```

## Architecture

```
RViz2 (ArmControlPanel)
    │  mit_enable / mit_disable / mit_zero / arm_send_enable / arm_poll_enable
    ▼
MoveIt2 → JointTrajectoryController (ros2_control)
                └── ArmHardwareInterface  (ros2_control plugin)
                        └── ArmController
                                └── CanBridge → USB-CAN-A (2 Mbps) → motors
```

`write()` sends an MIT command to each motor and immediately waits for the response (synchronous). States are updated in `read()` on the next cycle.

## Motors

| ID | Model | CAN frame | Inverted | kp_cmd | kd_cmd |
|----|-------|-----------|----------|--------|--------|
| 1 | AK45-36 | standard 11-bit | yes | 15.0 | 0.5 |
| 2 | AK60-39 | extended 29-bit | yes | 0.5 | 0.1 |
| 3 | AK45-36 | standard 11-bit | yes | 15.0 | 0.5 |
| 4 | AK45-36 | standard 11-bit | no | 15.0 | 0.5 |
| 5 | AK45-10 | standard 11-bit | yes | 15.0 | 0.5 |
| 6 | AK45-10 | standard 11-bit | no | 15.0 | 0.5 |
| 7 | AK40-10 | standard 11-bit | no | 15.0 | 0.5 (gripper) |

Motor 2 (AK60-39) uses extended CAN frames: MIT command `CAN ID = 0x802`, feedback `CAN ID = 0x2902`, disable `CAN ID = 0xF02`.

Parameter ranges and default gains are defined per-model in `ArmController.hpp` (`MotorPresets` namespace).

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `joint_states` | `sensor_msgs/JointState` | pub | Position/velocity from motor feedback |
| `mit_enable` | `std_msgs/Int32` | sub | Enable motors: `0` = all, `1–7` = single |
| `mit_disable` | `std_msgs/Int32` | sub | Disable motors: `0` = all, `1–7` = single |
| `mit_zero` | `std_msgs/Int32` | sub | Zero reference: `0` = all, `1–7` = single |
| `arm_send_enable` | `std_msgs/Bool` | sub | Enable/disable sending MIT commands |
| `arm_poll_enable` | `std_msgs/Bool` | sub | Enable/disable polling motor states |

## Build and run

```bash
cd ~/ros2_projekt/ros2_robotic_arm
colcon build
source install/setup.bash
ros2 launch arm6dof main.launch.py
```

Hardware interface opens `/dev/ttyUSB0`. The launch file also configures baud rate via `stty` before starting nodes.

## Manual motor control

```bash
# Enable all motors
ros2 topic pub --once /mit_enable std_msgs/msg/Int32 "{data: 0}"

# Enable motor 3 only
ros2 topic pub --once /mit_enable std_msgs/msg/Int32 "{data: 3}"

# Disable motor 2
ros2 topic pub --once /mit_disable std_msgs/msg/Int32 "{data: 2}"

# Zero all motors
ros2 topic pub --once /mit_zero std_msgs/msg/Int32 "{data: 0}"

# Enable sending commands
ros2 topic pub --once /arm_send_enable std_msgs/msg/Bool "{data: true}"
```

Alternatively, use the **ArmControlPanel** in RViz — it provides per-motor Enable / Disable / Zero buttons and global Send/Poll toggles.

## Direct serial tool (no ROS2)

```bash
python3 tools/trajectory.py
```

Sends MIT frames directly over serial, bypassing ros2_control. Useful for testing individual motors. Edit `CAN_PORT`, `MOTOR_IDS`, `POS_A`/`POS_B` at the top of the file.

## Dependencies

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-moveit
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
```

## Hardware

- **Waveshare USB-CAN-A** — CH340, binary serial framing, 2 Mbps
- **CAN protocol:** standard 11-bit MIT frames for most motors; extended 29-bit frames for AK60-39

## Project status

| Phase | Description | Status |
|-------|-------------|--------|
| 1 | ROS2 workspace, C++ node, launch file | ✅ |
| 2 | URDF 7-DOF, TF2, RViz2 visualization | ✅ |
| 3 | CanBridge (std + ext frames), ArmController MIT-only | ✅ |
| 4 | ros2_control Hardware Interface, MoveIt2 integration | ✅ |
| 5 | 7-motor support, AK60-39 extended frame protocol | ✅ |
| 6 | RViz ArmControlPanel (Qt plugin) | ✅ |
| 7 | Motion execution with real hardware | ⏳ |
| 8 | Gazebo simulation | ⏳ |
| 9 | Tests, CI/CD, Docker | ⏳ |

## Robot

7-DOF manipulator (6 arm joints + gripper). CAD models in `urdf/meshes/`.
