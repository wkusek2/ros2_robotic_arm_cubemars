# ROS2 Robotic Arm 6-DOF

Projekt edukacyjny budowy systemu sterowania manipulatorem 6-stopniowym w ROS2 Humble.

## Stack technologiczny

- **ROS2 Humble**
- **C++17** (węzły, kontrolery, komunikacja CAN)
- **Python** (launch files)
- **Gazebo Fortress** (symulacja — planowane)
- **MoveIt2** (planowanie ruchu — planowane)

## Struktura projektu

```
ros2_robotic_arm/
└── src/
    └── arm6dof/
        ├── src/arm/
        │   ├── armnode.cpp         # Węzeł ROS2: trajektoria, subskrybenty, canLoop
        │   ├── ArmController.hpp   # Interfejs kontrolera silników
        │   ├── ArmController.cpp   # Protokoły VESC CAN i MIT, receiveAny()
        │   ├── CanBridge.hpp       # Warstwa komunikacji Waveshare USB-CAN-A
        │   └── CanBridge.cpp       # Ramki seryjne, poll(), send/receive
        ├── msg/
        │   ├── MITFeedback.msg     # Feedback jednego silnika MIT
        │   └── MITFeedbackArray.msg# Feedback wszystkich silników (tablica)
        ├── launch/main.launch.py   # Launch file
        ├── urdf/arm6dof.urdf       # Opis robota
        └── urdf/meshes/            # Modele 3D STL
```

### Architektura węzła

```
ROS2 spin (subskrybenty + timery)
    ├── arm/mit_ctrl    → mitEnable / mitDisable / mitZero
    ├── arm/mit_hw_cmd  → sendMIT (bezpośrednia komenda)
    └── timer 50 Hz     → TrapezoidalProfile → sendMIT (trajektoria)

canLoop (osobny wątek, poll() na FD)
    └── receiveAny()
            ├── SERVO → joint_states + arm/diagnostics
            └── MIT   → arm/mit_feedback (MITFeedbackArray)

ArmController → CanBridge → USB-CAN-A (2 Mbps) → silniki AK45-36
```

## Topici

| Topic | Typ | Kierunek | Opis |
|-------|-----|----------|------|
| `arm/mit_ctrl` | `Float64MultiArray` | sub | `[motor_id, cmd]` — 1=enable, 0=disable, 2=zero |
| `arm/mit_hw_cmd` | `Float64MultiArray` | sub | `[motor_id, p, v, kp, kd, tau]` — bezpośrednia komenda MIT |
| `joint_states` | `JointState` | pub | Pozycja/prędkość/moment z feedbacku VESC |
| `arm/diagnostics` | `DiagnosticArray` | pub | Temperatura sterownika silnika |
| `arm/mit_feedback` | `MITFeedbackArray` | pub | Pełny feedback MIT wszystkich silników |

## Status projektu

| Faza | Opis | Status |
|------|------|--------|
| 1 | Workspace ROS2, węzeł C++, launch file | ✅ |
| 2 | URDF 6-DOF, TF2, wizualizacja RViz2 | ✅ |
| 3 | CanBridge (protokół binarny Waveshare), ArmController, feedback serwo (pos/vel/cur/temp), sterowanie prądem i pozycją, osobny wątek CAN z `poll()`, `joint_states` + `arm/diagnostics` | ✅ |
| 3+ | Tryb MIT (CubeMars): enable/disable/zero, `sendMIT`, feedback MIT, custom msg `MITFeedbackArray`, trajektoria trapezoidalna w C++ 50 Hz | ✅ |
| 4 | Symulacja Gazebo Fortress | ⏳ |
| 4 | ros2_control, Hardware Interface, kontrolery | ⏳ |
| 5 | Kinematyka FK/IK, parametry DH, KDL | ⏳ |
| 6 | MoveIt2, planowanie ruchu, unikanie kolizji | ⏳ |
| 7 | Action server/client, LifecycleNode, QoS | ⏳ |
| 8 | Testy, CI/CD, diagnostyki, Docker | ⏳ |

## Uruchomienie

```bash
# Zbuduj workspace
cd ~/ros2_robotic_arm
colcon build --packages-select arm6dof

# Załaduj środowisko (wymagane po każdym buildzie)
source install/setup.bash

# Uruchom węzeł
ros2 launch arm6dof main.launch.py
```

### Komendy sterowania

```bash
# Enable silnika 1 w trybie MIT
ros2 topic pub --once /arm6dof/arm/mit_ctrl std_msgs/msg/Float64MultiArray "{data: [1, 1]}"

# Zero enkodera silnika 1
ros2 topic pub --once /arm6dof/arm/mit_ctrl std_msgs/msg/Float64MultiArray "{data: [1, 2]}"

# Bezpośrednia komenda MIT: silnik 1, pozycja=0 rad, kp=5, kd=0.5
ros2 topic pub --once /arm6dof/arm/mit_hw_cmd std_msgs/msg/Float64MultiArray "{data: [1, 0.0, 0.0, 5.0, 0.5, 0.0]}"

# Podgląd feedbacku MIT
ros2 topic echo /arm6dof/arm/mit_feedback
```

## Wymagania

```bash
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-diagnostic-msgs
```

## Sprzęt

- **Waveshare USB-CAN-A** (CH340, protokół binarny, baudrate 2 Mbps, port `/dev/ttyUSB*`)
- **Silniki CubeMars AK45-36** (VESC firmware + MIT mode)
  - POLE_PAIRS=14, GEAR_RATIO=36, Kt=2.009 Nm/A
  - Zakres MIT: pozycja ±12.5 rad (16-bit), prędkość ±50 rad/s (12-bit), moment ±18 Nm (12-bit)
- **Protokół CAN:**
  - Tryb serwo (VESC): extended 29-bit, `CMD_SET_POS`=4, `CMD_SET_CURRENT`=1, feedback 0x29xx ~150 Hz
  - Tryb MIT: standard 11-bit, 8-bajtowe ramki z upakowanymi polami float→uint

## Robot

Manipulator 6-DOF zaprojektowany w CAD. Modele 3D w formacie STL.
