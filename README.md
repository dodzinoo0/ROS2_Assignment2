# ROS 2 Robotic Arm — URDF, Gazebo Simulation & Arduino Hardware Control

A complete ROS 2 workspace (tested on **ROS 2 Lyrical** on Ubuntu WSL 2) implementing a 4-DOF robotic arm with URDF modelling, synchronized **Gazebo (`gz sim`) + RViz2** simulation, a `JointTrajectory` publisher and Tkinter slider GUI, and a **USB Serial (`pyserial`)** hardware bridge driving real servo motors via an **Arduino Uno / Nano / Mega**.

---

## 1. Robotic Arm Model & Dimensions

Kinematic chain:

```text
world -> (fixed) -> base_link -> joint1 (yaw) -> L1 -> joint2 (pitch) -> L2 -> joint3 (pitch) -> L3 -> joint4 (pitch) -> end_effector
```

| Link | Dimension | Geometry in URDF | Joint | Type & Axis | Limits |
| :--- | :---: | :--- | :--- | :--- | :---: |
| `base_link` | **8 cm** | Box `0.10 × 0.10 × 0.08 m` | `world_to_base` | `fixed` | — |
| `L1` | **12 cm** | Cylinder `r=0.025 m, h=0.12 m` | `joint1` | `revolute` (Z) | `[-1.5708, +1.5708]` rad |
| `L2` | **15 cm** | Box `0.030 × 0.030 × 0.15 m` | `joint2` | `revolute` (Y) | `[-1.5708, +1.5708]` rad |
| `L3` | **9 cm** | Box `0.025 × 0.025 × 0.09 m` | `joint3` | `revolute` (Y) | `[-1.5708, +1.5708]` rad |
| `end_effector` | **6 cm** | Box `0.020 × 0.040 × 0.06 m` | `joint4` | `revolute` (Y) | `[-1.5708, +1.5708]` rad |

Every link includes `<visual>`, `<collision>`, and `<inertial>` tags.

---

## 2. System Architecture

```text
  [trajectory_publisher]  or  [arm_gui (Sliders)]
                      │
                      ▼  trajectory_msgs/msg/JointTrajectory
          /arm_controller/joint_trajectory
             │                        │
             ▼                        ▼
  [gz_trajectory_adapter]     [serial_bridge_node]
             │                        │  USB Serial (115200 baud, CSV)
             ▼  ros_gz_bridge         ▼
      [Gazebo (gz sim)]          [Arduino Uno/Nano/Mega]
             │                        │  PWM (Servo.h)
             ▼  /joint_states         ▼
  [robot_state_publisher]       [Servo 1 & Servo 2]
             │  /tf
             ▼
          [RViz2]
```

One command published to `/arm_controller/joint_trajectory` simultaneously moves the arm in **Gazebo**, **RViz2**, and the **physical servos**.

---

## 3. Repository Structure

```text
.
├── src/
│   ├── arm_description/        # URDF (robotic_arm.urdf), RViz2 config, display launch
│   ├── arm_gazebo/             # Gazebo world SDF, ros_gz_bridge YAML, sim & full pipeline launches
│   ├── arm_controller/         # gz_trajectory_adapter, trajectory_publisher, arm_gui (Tkinter)
│   └── arm_hardware_bridge/    # serial_bridge_node (ROS 2 -> USB serial via pyserial)
├── firmware/
│   └── arm_servo_controller/
│       └── arm_servo_controller.ino   # Arduino Uno/Nano/Mega PWM servo firmware
└── README.md
```

---

## 4. Dependencies & Installation (ROS 2 Lyrical on Ubuntu / WSL 2)

```bash
sudo apt update
sudo apt install -y \
  ros-lyrical-ros-gz \
  ros-lyrical-robot-state-publisher \
  ros-lyrical-rviz2 \
  python3-serial \
  python3-tk \
  python3-colcon-common-extensions
```

---

## 5. Build Instructions

Inside your WSL Ubuntu terminal, navigate to the workspace root and build with `colcon`:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **Tip for WSL users:** If you cloned this repo on a Windows drive (`/mnt/e/...`), copying or cloning it into your Linux home directory (e.g. `~/arm_ws`) gives noticeably faster `colcon build` times.

---

## 6. Running the Project

Always source the workspace first in each terminal:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

### A. Visualize URDF in RViz2 only
```bash
ros2 launch arm_description display.launch.py
```

### B. Launch Gazebo (`gz sim`) + RViz2 Simulation
```bash
ros2 launch arm_gazebo sim.launch.py
```

### C. Run the Automated `JointTrajectory` Publisher Demo
With Gazebo + RViz2 running, open a second terminal and run:
```bash
ros2 run arm_controller trajectory_publisher
```
The arm will smoothly execute a looping multi-pose sequence in both Gazebo and RViz2.

### D. Run the Full Pipeline (GUI Sliders + Gazebo + RViz2 + Arduino Servos)

1. **Test without Arduino connected (`dry_run:=true`):**
   ```bash
   ros2 launch arm_gazebo full_pipeline.launch.py dry_run:=true
   ```
2. **Run with real Arduino connected:**
   ```bash
   ros2 launch arm_gazebo full_pipeline.launch.py serial_port:=/dev/ttyACM0 baudrate:=115200
   ```
   *(Use `/dev/ttyUSB0` if your Arduino uses a CH340/FTDI USB-to-serial chip.)*

You can also run the slider GUI or hardware bridge standalone:
```bash
ros2 run arm_controller arm_gui
ros2 run arm_hardware_bridge serial_bridge_node --ros-args -p serial_port:=/dev/ttyACM0
```

---

## 7. Hardware Communication Details

### Transport & Message Format
- **Transport:** USB Serial using Python `pyserial` at **115200 baud** (8N1, newline-delimited ASCII).
- **Message Format (ROS 2 → Arduino):**
  ```text
  S,<joint1_deg>,<joint2_deg>\n
  ```
  Example: `S,90,135\n` commands Servo 1 (`joint1`) to `90°` and Servo 2 (`joint2`) to `135°`.
  The firmware also accepts up to 4 comma-separated angles (`S,<j1>,<j2>,<j3>,<j4>\n`) if additional servos are attached.
- **Acknowledgement (Arduino → ROS 2):**
  ```text
  OK,<joint1_deg>,<joint2_deg>\n
  ```
- **Angle Conversion & Clamping:**
  Joint angles in radians `[-1.5708, +1.5708]` (`[-π/2, +π/2]`) are linearly mapped to `[0, 180]` degrees (`0 rad -> 90°` center) and clamped strictly to `[0, 180]` on both the ROS 2 node and the Arduino firmware before writing PWM.

### Wiring / Pin Mapping (Arduino Uno / Nano / Mega)

| Board Pin | Connects To |
| :--- | :--- |
| **D9** (PWM) | Servo 1 signal wire (`joint1` — base rotation) |
| **D10** (PWM) | Servo 2 signal wire (`joint2` — shoulder pitch) |
| **D11** (PWM) | Servo 3 signal wire (`joint3` — elbow pitch, optional) |
| **D6** (PWM) | Servo 4 signal wire (`joint4` — wrist pitch, optional) |
| **GND** | Common ground shared with external 5V servo power supply |

> **Power Warning:** Power the servo motors from an external 5V power supply (e.g. 5V 2A+) rather than the Arduino 5V pin, and connect the external supply's **GND** directly to the Arduino **GND**.

### Passing the Arduino USB Port into WSL 2 (`usbipd-win`)

1. Upload `firmware/arm_servo_controller/arm_servo_controller.ino` to your Arduino using the Arduino IDE on Windows.
2. In an **Administrator PowerShell** on Windows, install `usbipd-win` (if not already installed):
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```
3. List USB devices and attach the Arduino to WSL:
   ```powershell
   usbipd list
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```
4. Inside WSL Ubuntu, verify the serial device appeared and grant permissions:
   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   sudo chmod 666 /dev/ttyACM0
   ```
   *(To avoid running `chmod` every time, add your user to the `dialout` group: `sudo usermod -aG dialout $USER` and restart WSL.)*
