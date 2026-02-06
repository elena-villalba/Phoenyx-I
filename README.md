<p align="center">
  <img src="resources/PHOENYX-1-logo-recortado.png" alt="Project Logo"/>
</p>


> ### **🚀 From Mars to the Lab:**
>
> 🤖 **Meet PHOENYX-I**, an autonomous exploration rover inspired by **NASA’s designs**, bringing **cutting-edge robotics** from the lab to Mars-like terrains.
>
> 🧠 Built on **ROS 2**, Phoenyx I combines **real-time perception**, **LiDAR-based navigation**, and **visual localization**, enabling fully autonomous operation in both simulation and real-world environments.
> 
> 🏆 **Winner of Best Overall Rover & Design Excellence** at **Sener-CEA's Bot Talent competition**.

---

 ## 👀 Watch PHOENYX-I in action 
 
<p align="center">

[![Phoenyx I – Autonomous Rover Demo](https://img.youtube.com/vi/FE0RjScWbhM/maxresdefault.jpg)](https://www.youtube.com/watch?v=FE0RjScWbhM)

</p>


| 🔍 Perception Task | 🛣️ Control Task | 📍 Guidance Task |
|---|---|---|
| [![Perception](https://img.youtube.com/vi/iHNUQLfxfGA/maxresdefault.jpg)](https://youtube.com/shorts/iHNUQLfxfGA) | [![Control](https://img.youtube.com/vi/iHNUQLfxfGA/maxresdefault.jpg)](https://youtube.com/shorts/iHNUQLfxfGA)  | [![Guidance](https://img.youtube.com/vi/iHNUQLfxfGA/maxresdefault.jpg)](https://youtube.com/shorts/iHNUQLfxfGA)|
| <p align="center">Real-time digit and color recognition.</p> | <p align="center">Autonomous corridor navigation using only 2D LiDAR.</p> | <p align="center">Localization and waypoint navigation using ArUco markers.</p> |


## 🧭 Project Overview

**PHOENYX-I** is an award-winning autonomous rover developed by undergraduate students from **PUCRA**, the robotics association at the **Polytechnic University of Catalonia**.

This repository contains the complete **ROS 2 autonomy stack** used on the robot, including simulation environments, perception pipelines, control nodes and navigation modules deployed both in competition and real-world operation.

Built on top of the [NASA JPL Open Source Rover](https://github.com/nasa-jpl/open-source-rover), **PHOENYX-I** extends the base platform with a modular autonomy architecture optimized for embedded computing on a **Raspberry Pi 4B (4 GB RAM)**.

The system integrates multiple sensors to enable autonomous operation:

- 🎥 **Intel RealSense D435i** — RGB-D perception and visual detection  
- 📡 **RPLidar** — 2D LiDAR for navigation and obstacle detection  
- 🧭 **MPU6050 IMU** — orientation estimation and motion stabilization  
- 🔋 **INA260** — power monitoring and battery supervision  

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue"/>
  <img src="https://img.shields.io/badge/Ubuntu-22.04-orange"/>
  <img src="https://img.shields.io/badge/Hardware-RaspberryPi4B-red"/>
  <img src="https://img.shields.io/badge/License-MIT-green"/>
</p>

## 📦 Jump to:

- [🚀 Quick Start ](#-quick-start)
- [📂 Repo Structure](#-repository-structure)
- [🚦 How to Run](#-how-to-run-the-system) 
- [🏁 Competition Results](#-competition-results)
- [🤝 Want to Collaborate?](#-want-to-collaborate)
- [🌐 Join & Follow Us](#-join--follow-us)

## 🚀 Quick Start 

### PC (Development & Visualization)

The PC is used as a **development and visualization environment**, either to:
- **run simulations in Gazebo**, or
- **visualize and monitor the real robot** (e.g. using RViz) while it is running.

To set up the Phoenyx I PC environment on **Ubuntu 22.04 LTS**, use the provided installation script.

1. Download the installation script:
   ```bash
   $ wget https://raw.githubusercontent.com/PUCRA/Phoenyx-I/refs/heads/main/scripts/install_pc.sh
   ```

2. Make the script executable:
   ```bash
   $ chmod +x install_pc.sh
   ```

3. Run the script:
   ```bash
   ./install_pc.sh
   ```

### Raspberry Pi

To run Phoenyx I on the real robot, you first need to set up the onboard Raspberry Pi.

👉 **Follow the complete Raspberry Pi setup guide here:**  
➡️ [`docs/raspberry_pi_setup.md`](docs/raspberry_pi_setup.md)

### Steering Servo Calibration

Before operating the rover on real hardware, the steering servos **must be calibrated** to ensure all wheels are correctly aligned and the full steering range is used.

This procedure is **only required once**, or whenever a steering servo or corner assembly is replaced.

👉 **Follow the complete steering servo calibration guide here:**  
➡️ [`docs/steering_servo_calibration.md`](docs/steering_servo_calibration.md)
  

## 🚦 How to Run the System

### 🧪 In Simulation 

#### Empty Worlds
```bash
# Launch an empty world on gazebo with the rover model
ros2 launch osr_gazebo empty_world.launch.py

# Launch an empty world on gazebo and rviz with the simplified model of the rover model
ros2 launch osr_gazebo empty_world_simplified.launch.py

# Launch rviz with the rover model
ros2 launch osr_gazebo rviz.launch.py

# Launch rviz with the simplified model of the rover model
ros2 launch osr_gazebo rviz_simplified.launch.py 
```

### Move With a Controller
```bash
# With an open gazebo world with the rover model you can use a controller to move it
ros2 launch osr_bringup joystick.launch.py mode:=sim
```

> [!NOTE]
> For details on the controller layout, how to change the joystick mapping, and a deeper explanation of how the teleoperation pipeline works, see: [Joystick Teleoperation Documentation](docs/joystick_teleoperation.md)

#### Control Task (Without Nav2)
This mode is used to evaluate the rover’s control and navigation behavior inside a **maze environment**.
Three maze worlds are provided, each with different layouts and difficulty levels.

Available maze worlds:
- maze_1.world — basic layout, suitable for tuning and testing.
- maze_2.world — intermediate complexity.
- maze_3.world — advanced maze for full control evaluation.

```bash
# Terminal 1 - Launch the simulation world with the desired maze
ros2 launch osr_bringup maze_simulation.launch.py maze:=maze_1.world
# or
ros2 launch osr_bringup maze_simulation.launch.py maze:=maze_2.world
# or
ros2 launch osr_bringup maze_simulation.launch.py maze:=maze_3.world

# Terminal 2 - Run the control challenge node
ros2 run osr_control_challenge maze_navigation
```

#### Guidance Task

```bash
# Terminal 1 - Launch simulation world
ros2 launch osr_gazebo circuito_arucos.launch.py

# Terminal 2 - Launch SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 - Launch Nav2
ros2 launch planificador planificador_launch.py use_sim_time:=true

# Terminal 4 - Launch Brain
ros2 run guiado brain_guiado.py use_sim_time:=true

# Terminal 5 - Publish a true on topic /aruco_scan
ros2 topic pub --once /aruco_scan std_msgs/Bool "{data: true}"

```

### 🤖 On Real Robot 

#### To control with a controller
```bash
ros2 launch osr_bringup joystick.launch.py
```

#### For percepcion task
``` bash
ros2 launch prueba2_percepcion.launch.py 
```
#### For control task 
```bash
# Terminal 1 
ros2 launch control control.launch.py

# Terminal 2
ros2 launch control planificador.launch.py
```
#### For guiado task 
```bash
ros2 launch guiado guiado.launch.py
```

> [!note]
> Press A on the joystick to start autonomous mode `/joy` topic.
>
>  See [Orbbec ROS 2 README](https://github.com/PUCRA/Phoenyx/tree/main/OrbbecSDK_ROS2) for camera setup.


## 📁 Repository Structure

```bash
├── src/
│   ├── osr_bringup/            # Basic launch files and configuration for the OSR
│   ├── osr_control/            
│   ├── osr_control_challenge/  # Code for the control task
│   ├── osr_gazebo/             # Simulation environment for Gazebo
│   ├── osr_interfaces/         # Custom ROS messages
│   ├── percepcion/             # Image recognition, color and digit detection
│   ├── phoenyx_nodes/          # Multiple nodes for different tasks and applications
│   ├── planificador/           # Package for custom launch and YAML configuration
│   ├── guiado/                 # SLAM-based localization and waypoint navigation
│   ├── control/                
│   ├── datos/                  
│   └── final/                  
│
├── LICENSE.md
├── JPL_NASA_LICENCE.txt
└── README.md

```

## 🏁 Competition Results

- 🥇 **First Place Overall – Bot Talent 2025**  
- 🧠 **Awarded for Best Robot Design**
  
## License

This main project, including all contributions by PUCRA and its collaborators, is licensed under the **[MIT License](LICENSE)**.

Parts of this project derived from the work of the **Jet Propulsion Laboratory (JPL) of NASA** are covered by the **[Apache License, Version 2.0](JPL_NASA_License.txt)**. Please refer to the `JPL_NASA_License.txt` file for full terms and original attributions.

## 🤝 Want to Collaborate?

If you're interested in contributing code, improving documentation, or developing new features, feel free to check our [CONTRIBUTING](.github/CONTRIBUTING.md) page!

## 🌐 Join & Follow Us

Stay connected with PUCRA and follow our journey:

[![email](https://img.shields.io/badge/Email-D14836?logo=gmail&logoColor=white)](mailto:pucra.eebe@upc.edu) 
[![LinkedIn](https://img.shields.io/badge/LinkedIn-%230077B5.svg?logo=linkedin&logoColor=white)](https://www.linkedin.com/company/pucra-upcc/posts/?feedView=all)
[![Instagram](https://img.shields.io/badge/Instagram-%23E4405F.svg?logo=Instagram&logoColor=white)](https://www.instagram.com/pucra.upc/)
[![YouTube](https://img.shields.io/badge/YouTube-%23FF0000.svg?logo=YouTube&logoColor=white)](https://www.youtube.com/@pucraupc) 

<p align="center">
  <img src="resources/logo.png" alt="Project Logo"/>
</p>

