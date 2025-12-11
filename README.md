# AMR Demo Project (ROS 2 Jazzy)

## 📖 Introduction
This is an Autonomous Mobile Robot (AMR) simulation project developed using ROS 2 Jazzy and C++.
The primary goal of this project is to demonstrate robot development capabilities within the modern ROS 2 architecture, covering node design, physical simulation, autonomous navigation (Nav2), and frontend-backend integration (Web UI).

## ✨ Key Features
* **Physical Simulation:** Robot model defined using URDF/Xacro with dynamic simulation in Gazebo Harmonic.
* **Autonomous Obstacle Avoidance:** Implemented C++ node that processes LaserScan data to control `cmd_vel` for real-time collision avoidance.
* **Modern Development:** Optimized for ARM64 architecture (Apple Silicon) using the latest ROS 2 Jazzy LTS release.

## 🛠️ Development Environment
* **Hardware:** MacBook Pro (M1 Chip / ARM64)
* **Virtualization:** Parallels Desktop 19
* **OS:** Ubuntu 24.04 LTS (Noble Numbat) - ARM64
* **Middleware:** ROS 2 Jazzy Jalisco
* **Simulation:** Gazebo Harmonic

## 🚀 Roadmap
Current development status and future plans:

### Phase 1: Infrastructure & Control (Completed)
- [x] Establish ROS 2 C++ Package structure
- [x] Design AMR URDF model (including Lidar and IMU sensor configuration)
- [x] Configure Gazebo Harmonic simulation environment
- [x] Implement basic Obstacle Avoidance Node
![amr_20151211](https://github.com/user-attachments/assets/ad8a9938-2f54-471a-92c5-0f35cd1e20ab)

### Phase 2: Perception & Localization (Current Focus)
- [ ] **SLAM Integration:** Integrate `slam_toolbox` for Simultaneous Localization and Mapping.
- [ ] **Sensor Fusion:** Use `robot_localization` (EKF) to fuse IMU and Odometry data for improved odometry accuracy.
- [ ] **TF2 Setup:** Refine Coordinate Transforms tree.

### Phase 3: Autonomous Navigation (Planned)
- [ ] **Nav2 Integration:** Configure Navigation 2 Stack (Planner, Controller, Behavior Trees).
- [ ] **Waypoints Following:** Implement multi-point patrol functionality.

### Phase 4: System Integration & Interface (Planned)
- [ ] **Web UI:** Develop a web control interface using `rosbridge_suite` and React/Vue (display map, send navigation goals).
- [ ] **Dockerization:** Create Dockerfile and DevContainer to support cross-platform (x86/ARM) rapid deployment.
- [ ] **CI/CD:** Setup GitHub Actions for automated building and GTest unit testing.
