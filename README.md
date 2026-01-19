# Uni-Bot: Autonomous Mobile Robot Platform

<div align="center">

**A ROS 2-based autonomous mobile robot featuring robust localization, mapping (SLAM), and patrol navigation.**

*Developed by the Unipod Robotics Team*

---

**Onyenweaku Chibueze** *(Lead Engineer/Robotics Software Engineer)* · **Osuntuyi Olumide George** *(Mechanical Engineer/Product Designer)*
**Fred Orunto** *(Electrical Engineer)* · **Orobosa** *(Electrical Engineer)* · **Kwuelum Eniola** *(Product Designer)* · **Josiah Adesola** · **Ebo Anthony** *(Mechanical/Electrical Engineer)* · **Gboyega** *(Software Engineer)*

---

</div>

## Abstract

Uni-Bot is an autonomous mobile robot platform designed for indoor navigation, mapping, and patrol applications. The system leverages a Raspberry Pi 5 as the main compute unit, integrated with an ESP32 for real-time motor control. 

The robot successfully demonstrates **autonomous navigation using RPLIDAR A2 and wheel encoders**, enabling precise Simultaneous Localization and Mapping (SLAM). Additionally, it features an ArUco marker-based sequential patrol mode for waypoint navigation in structured environments.

---

## 1. Introduction

### 1.1 Motivation

Autonomous mobile robots are increasingly deployed for inspection, delivery, and monitoring. This project demonstrates a cost-effective yet powerful platform capable of:
- **High-fidelity Mapping:** Using LiDAR SLAM to create accurate 2D maps of the environment.
- **Autonomous Localisation:** Fusing encoder odometry with IMU and LiDAR data for robust state estimation.
- **Perception-based Navigation:** Utilizing depth cameras and fiducial markers for task specific maneuvers.

### 1.2 Objectives

1. Design and fabricate a robust 4-wheel differential drive platform.
2. Implement a modular ROS 2 software architecture.
3. **Achieve autonomous navigation and mapping** using LiDAR and encoder fusion.
4. Develop ArUco marker-based sequential patrol for specific inspection tasks.

---

## 2. System Architecture

### 2.1 Hardware Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 5                          │
│                    (Main Compute Unit)                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │  ROS 2   │  │  SLAM    │  │  ArUco   │  │   Nav    │        │
│  │  Core    │  │ Toolbox  │  │ Detector │  │  Stack   │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└────────────────────────┬────────────────────────────────────────┘
                         │ Serial/USB
┌────────────────────────┴────────────────────────────────────────┐
│                         ESP32                                    │
│              (Real-time Motor Controller)                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                       │
│  │   PID    │  │ Encoder  │  │  Motor   │                       │
│  │ Control  │  │ Reading  │  │ Commands │                       │
│  └──────────┘  └──────────┘  └──────────┘                       │
└────────────────────────┬────────────────────────────────────────┘
                         │
    ┌────────────────────┼────────────────────┐
    ▼                    ▼                    ▼
┌────────┐          ┌────────┐          ┌────────┐
│Motor 1 │          │Motor 2 │          │Motor 3,4│
│BTS7960 │          │BTS7960 │          │BTS7960  │
└────────┘          └────────┘          └────────┘
```

### 2.2 Software Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      ROS 2 Jazzy Jalisco                        │
├─────────────────────────────────────────────────────────────────┤
│  ubot_description    │  Robot URDF/Xacro models                 │
│  ubot_control        │  Hardware interface (ros2_control)       │
│  ubot_bringup        │  Launch files, configs, worlds           │
│  ubot_aruco          │  ArUco detection & patrol navigation     │
└─────────────────────────────────────────────────────────────────┘
           │                    │                    │
           ▼                    ▼                    ▼
    ┌────────────┐      ┌────────────┐      ┌────────────┐
    │SLAM Toolbox│      │ Nav2 Stack │      │   Gazebo   │
    │  (Mapping) │      │(Navigation)│      │(Simulation)│
    └────────────┘      └────────────┘      └────────────┘
```

---

## 3. Methodology

### 3.1 Autonomous Navigation & Mapping (SLAM)

The core autonomy is achieved through the fusion of:
- **RPLIDAR A2:** Provides 360° laser scan data for obstacle detection and map building.
- **Wheel Encoders:** Provide high-resolution odometry feedback (position and velocity).
- **SLAM Toolbox:** Processes sensor data to generate real-time occupancy grid maps and correct localization drift.
- **Nav2 (Navigation 2):** (Future integration) Uses the generated map to plan paths and avoid dynamic obstacles.

### 3.2 ArUco Marker Patrol

For specific waypoint tasks without a map, the ArUco system implements a sequential state machine:

| State | Description |
|-------|-------------|
| **SEARCHING** | Robot rotates to find target marker |
| **APPROACH** | Visually servos towards marker using Proportional Control |
| **NEXT_MARKER** | Advances sequence upon reaching target |

Sequence: **Marker 0 → 1 → 2 → 3 → 4**

---

## 4. Hardware Components

### 4.1 Bill of Materials

#### Power System

| Component | Specification | Quantity | Link |
|-----------|---------------|:--------:|------|
| UPS HAT | X1200 Raspberry Pi 5, 5.1V 5A | 1 | [Link](https://www.dfrobot.com/product-2840.html) |
| BMS | 4S 16.8V 40A | 1 | [Link](https://hub360.cc/shop/0315-4s-40a-18650-balanced-bms-with-protection-board-12123) |
| Li-Ion Batteries | 18650 9900mAh 3.7V | 8 | [Link](https://hub360.com.ng/product/flat-head-18650-3-7v-9900mah-lipo-battery/) |
| Battery Holder | 18650 4S | 1 | Locally Sourced |
| Buck Converter | XL4015 DC-DC 5A | 1 | [Link](https://hub360.cc/shop/2840-xl4015-buck-module-17096) |
| Rectifier Diodes | UF5408 800V 3A | 5 | [Link](https://hub360.com.ng/product/uf5408/) |
| Rocker Switches | SPST 2-Pin | 2 | [Link](https://hub360.com.ng/product/switch/) |

#### Sensors

| Component | Specification | Quantity | Link |
|-----------|---------------|:--------:|------|
| Depth Camera | Intel RealSense D435 | 1 | Owned |
| LiDAR | Slamtec RPLIDAR A2 | 1 | [Link](https://www.dfrobot.com/product-1461.html) |
| Load Sensor | 20kg Load Cell | 1 | [Link](https://hub360.cc/shop/0585-20kg-load-cell-weight-sensor-12393) |
| GPS Module | NEO-M8N | 1 | Owned |
| IMU | MPU-9250 10-DOF | 1 | [Link](https://hub360.cc/shop/1247-mcu-117-mcu-10dof-mpu9250-ms5611-9-axis-10-dof-module-14484) |

#### Actuators & Drivers

| Component | Specification | Quantity | Link |
|-----------|---------------|:--------:|------|
| Motors | JGB520 Gearmotor 12V | 4 | [Link](https://hub360.com.ng/product/relay-module-2-channel/) |
| Motor Drivers | BTS7960 43A | 4 | [Link](https://hub360.cc/shop/1314-bts7960-43a-motor-driver-module-high-power-smart-control-14551) |
| Pi Cooler | Aluminum Heatsink + PWM Fan | 1 | [Link](https://hub360.cc/shop/2674-raspberry-pi-5-radiator-cooler-active-aluminum-heatsink-with-pwm-fan-for-pi-5) |

#### Processing Units

| Component | Specification | Quantity | Link |
|-----------|---------------|:--------:|------|
| Raspberry Pi 5 | 4GB RAM | 1 | Owned |
| ESP32 | Dual Core 30-Pin WROOM | 1 | Owned |
| Arduino Nano | ATmega328P | 1 | Owned |

#### Peripherals

| Component | Specification | Quantity | Link |
|-----------|---------------|:--------:|------|
| SD Card | SanDisk 32GB | 1 | [Link](https://hub360.cc/shop/1415-raspberry-pi-sd-card-32gb-14652) |
| HX711 Amplifier | Load Cell Interface | 1 | Locally Sourced |
| Breadboard PSU | 3.3V/5V Module | 2 | [Link](https://hub360.com.ng/product/breadboard-power-supply/) |
| Wheels | 80mm Rubber, 6mm Hex | 4 | 3D Printed |
| Chassis | Custom Frame | 1 | 3D Printed |

---

## 5. Software Implementation

### 5.1 ROS 2 Packages

| Package | Description |
|---------|-------------|
| `ubot_description` | URDF/Xacro robot model, meshes, RViz configs |
| `ubot_control` | ros2_control hardware interface for ESP32 |
| `ubot_bringup` | Launch files for simulation and real robot |
| `ubot_aruco` | ArUco marker detection and patrol navigation |

### 5.2 Building the Workspace

```bash
# Install dependencies
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup \
                 ros-jazzy-ros-gz libserial-dev

# Clone and build
cd ~/uni-bot
colcon build
source install/setup.bash
```

### 5.3 Running the Robot

**Simulation:**
```bash
ros2 launch ubot_bringup sim.launch.py
```

**Real Robot:**
```bash
ros2 launch ubot_bringup real_robot.launch.py
```

**ArUco Patrol:**
```bash
ros2 launch ubot_aruco aruco_patrol.launch.py
```

---

## 6. Results

### 6.1 Simulation Validation

The robot was validated in Gazebo Harmonic simulation environment with:
- Successful URDF loading and joint state publishing.
- Differential drive control via ros2_control.
- **Perfect autonomous navigation utilizing LiDAR and encoder fusion.**
- Reliable ArUco marker detection and following.

---

## 7. Conclusion

Uni-Bot demonstrates a successful integration of ROS 2 with custom hardware. The platform achieves **robust autonomous navigation using LiDAR SLAM and wheel encoders**, proving the efficacy of the differential drive design and sensor fusion algorithms. The addition of ArUco patrol capabilities further extends its utility for structured inspection tasks.

---

## 8. References

1. ROS 2 Documentation. https://docs.ros.org/en/jazzy/
2. OpenCV ArUco Module. https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
3. SLAM Toolbox. https://github.com/SteveMacenski/slam_toolbox
4. Gazebo Harmonic. https://gazebosim.org/

---

*Unipod Robotics Team © 2026*
