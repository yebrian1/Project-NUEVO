# Project NUEVO
![](/assets/NUEVO.png)

Group 4 - Sanjith, Mihir, Brian, Duc, Ivy, Nathan, Richard.

# Autonomous Navigation & Manipulation Robot

![Project Status: Completed](https://img.shields.io/badge/Status-Completed-brightgreen)
![Python](https://img.shields.io/badge/Language-Python_3.10+-blue)
![ROS2](https://img.shields.io/badge/Framework-ROS_2-orange)

<div align="center">
  *Placeholder: Highly recommend adding a GIF here of your robot detecting a face, grabbing an object, or avoiding an obstacle.*
</div>

## 📌 Project Overview

This project represents the complete software and hardware stack for an autonomous differential-drive robot designed for complex service and delivery tasks. The robot relies on a fusion of precise internal odometry and rich environmental sensor data (360° LiDAR and an RGB Camera) to navigate dynamic environments without the need for external positioning systems like GPS. 

It features a robust computer vision pipeline for traffic light compliance, customer identification (face/gender classification), and stop sign detection. Navigation is handled dynamically using Proportional-control wall following and Local Artificial Potential Fields (LAPF) for obstacle avoidance. Finally, it utilizes a multi-axis robotic arm (stepper-driven) with a servo-actuated gripper for physical manipulation and payload delivery.

The software architecture is built on a highly synchronous, non-blocking Python Finite State Machine (FSM), integrated with ROS 2, and containerized using Docker for deployment on a Raspberry Pi.

---

## ✨ Key Features

* **Dynamic Obstacle Avoidance (LAPF):** Implemented Local Artificial Potential Fields to continuously calculate attractive forces toward a goal and repulsive forces from obstacles, allowing the robot to smoothly navigate around unexpected objects.
* **Closed-Loop Wall Following:** Utilizes a P-controller (`FOLLOW_KP`) tied to lateral LiDAR distance readings to maintain a parallel trajectory alongside walls, adjusting wheel velocities dynamically to correct drift.
* **Advanced Computer Vision Pipeline:** * **Traffic Light Detection:** Pauses state execution until a green light is confirmed.
    * **Customer Classification:** Executes a multi-frame "majority vote" face scanning algorithm to classify customer profiles and dynamically adjust delivery drop-off distances.
    * **Sign Recognition:** Dynamically pans the camera servo while driving to scan for and halt at stop signs.
* **Precision LiDAR Alignment:** Calculates spatial relationships from point clouds to execute sub-centimeter halting distances and square the robot's heading perfectly perpendicular to flat walls.
* **Complex Kinematics & Manipulation:** Executes precision dead-reckoning turns and coordinates a multi-stage payload pickup/dropoff sequence using a stepper-driven elevator and servo gripper.

---

## 🛠️ Hardware Architecture

* **Brains:** Raspberry Pi (Dockerized ROS 2 environment, CV processing, Path Planning, LAPF computation)
* **Real-Time Controller:** Arduino (Motor control, hardware interrupts) communicating via UART.
* **Sensors:** * 360° 2D LiDAR (Obstacle detection, wall tracking, precision alignment)
    * RGB Camera (Vision classification)
    * High-Resolution Wheel Encoders (Odometry & Dead Reckoning)
* **Actuators:**
    * 2x DC Motors (Differential Drive)
    * 1x NEMA Stepper Motor (Arm elevation/Z-axis)
    * 2x Servo Motors (Gripper actuation & Camera panning mechanism)
* **Power Distribution:** 10-Cell 12.2V NiMH Battery with optically isolated step-down regulators to protect logic boards from inductive motor spikes.

---

## 🧠 Software Architecture (Finite State Machine)

The robot operates completely autonomously once initialized, governed by a non-blocking FSM running at a targeted Hz rate. Key mission states include:

1.  `WATCHING`: Actuates the camera to scan the environment and holds the robot's position until a green traffic light is detected.
2.  `DRIVE_WALL_FOLLOW`: Locks onto an adjacent wall using LiDAR and dynamically adjusts angular velocity to maintain a strict offset distance.
3.  `ALIGN_FRONT_WALL` & `FINAL_APPROACH`: Squares the robot to physical targets and executes precise millimeter-accurate docking to pick up payloads.
4.  `OBSTACLE_AVOIDANCE_MOVING`: Hands navigation control over to the LAPF algorithm to autonomously route around physical blockages toward a target coordinate.
5.  `POST_NAV_SCAN`: Pans the camera, executes the facial classification majority-vote logic, and recalculates the final delivery route based on the result.
6.  `WATCH_FOR_STOP_SIGN`: Moves toward the delivery zone while scanning for stop signs, triggering a timed halt upon detection.

---

## 🔬 Hardware Engineering & Lessons Learned

Building a robot bridging high-level ROS 2 software with raw electrical hardware provided significant systems engineering experience:

* **Thermal Management & System Stability:** Encountered severe UART degradation and system lockups during extended runs. Root-caused the communication dropouts to the Raspberry Pi overheating and thermally throttling under the heavy processing load of running Computer Vision, LAPF navigation, and ROS 2 concurrently. Solved this hardware limitation by implementing an aggressive active cooling strategy—maxing out the Pi's cooling fan—which stabilized the core temperature and completely resolved the UART errors.
* **Voltage Sag & Kinematic Drift:** Diagnosed "brownout" conditions where heavy servo loads dropped system voltage to 3.7V mid-run. Learned to implement software compensation for DC motor PWM under varying battery states, and physical power buffers to prevent stepper motor skipped steps.
* **UART Bandwidth Throttling:** Discovered that sending unthrottled velocity commands to the Arduino during P-control loops overwhelmed the serial buffer. Implemented state-variable rate limiting (`CMD_RATE_LIMIT_S`) to cap motor updates to 10Hz, drastically improving system stability.

## Overview

A modular two-wheel drive mobile robot platform designed for hands-on robotics education. Features customizable manipulators and a dual-layer control architecture for teaching embedded systems, ROS2, and mechatronics fundamentals.

## System Architecture

**Low-Level Control (Arduino)**
- Real-time motor control (DC, stepper, servo)
- GPIO, LEDs, and button inputs
- UART communication to Raspberry Pi

**High-Level Control (Raspberry Pi 5 + ROS2)**
- Decision-making and path planning
- Camera and GPS sensor processing
- ROS2 node orchestration

**Custom PCB**
- Integrates Arduino, motor drivers, and power management
- Standardized interface for educational reproducibility

## Repository Structure

```
├── firmware/       Arduino firmware and firmware-specific docs
├── nuevo_ui/       Raspberry Pi bridge + web UI
├── ros2_ws/        ROS2 workspace and Pi-side tests
├── tlv_protocol/   TLV type definitions, payload schemas, generators
├── NUEVO board/    PCB design files (schematics, layouts, BOM)
├── mechanical/     CAD files for chassis and manipulators
├── docs/           Cross-project architecture, protocol, and design docs
└── assets/         Shared repo assets

```



## Key Documents

| Document | Purpose |
|----------|---------|
| [docs/README.md](docs/README.md) | Cross-project documentation map and source-of-truth index |
| [docs/COMMUNICATION_PROTOCOL.md](docs/COMMUNICATION_PROTOCOL.md) | Current human-readable source of truth for protocol behavior, framing, and logical TLV design |
| [docs/DESIGN_GUIDELINES.md](docs/DESIGN_GUIDELINES.md) | Cross-project conventions, numbering rules, and protocol update workflow |
| [tlv_protocol/TLV_Payloads.md](tlv_protocol/TLV_Payloads.md) | Exact payload layouts and sizes |
| [firmware/README.md](firmware/README.md) | Arduino firmware overview, current features, and build instructions |
| [firmware/docs/README.md](firmware/docs/README.md) | Firmware subsystem documentation index |
| [NUEVO board/SPECIFICATIONS.md](NUEVO%20board/SPECIFICATIONS.md) | PCB hardware specifications |
| [main_autonomous_robot_overview.md](ros2_ws/src/robot/robot) | Robot overview, FSM, hardware, electrical specifications |

## Technologies

- **Embedded**: Arduino (C/C++)
- **High-Level**: ROS2 (Python/C++), Raspberry Pi 5
- **Communication**: UART serial protocol
- **Sensors**: Camera, GPS, encoders
- **Hardware**: Custom PCB, stepper/servo motors
