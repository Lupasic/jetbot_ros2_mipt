# JetBot ROS2 MIPT

A complete ROS2 implementation for JetBot differential drive robot with navigation capabilities, designed for educational and research purposes at MIPT.

## Overview

This project provides a full ROS2 stack for JetBot robots including:
- Hardware interface for differential drive control
- LIDAR integration (RPLidar)
- Navigation stack (Nav2)
- SLAM capabilities (slam_toolbox)
- Joystick control support
- Docker-based deployment

## Prerequisites

- NVIDIA Jetson platform (tested on Jetson Nano)
- Docker with NVIDIA runtime support
- ROS2 Humble (via Docker)
- Connected hardware:
  - JetBot motors via `/dev/ttyMOTOR`
  - RPLidar via `/dev/ttyLIDAR`
  - USB camera via `/dev/video0`
  - Gamepad controller via `/dev/input/js0`

## Docker Architecture

This project uses a two-stage Docker build process:

### 1. Base Image (`docker_configs/Dockerfile_base`)
- **Purpose**: Contains general-purpose ROS2 packages and dependencies
- **Contents**: ROS2 Humble core, Navigation2, RViz2, RQT tools, build tools
- **Build location**: Recommended to build on powerful AMD64 host machine
- **Usage**: Foundation for all JetBot applications

### 2. Application Image (`Dockerfile`)
- **Purpose**: Adds specific packages for JetBot functionality
- **Contents**: Additional ROS2 packages (xacro, twist_mux, joystick drivers, etc.)
- **Build location**: Can be built on JetBot or host machine
- **Usage**: Ready-to-run JetBot container

## Quick Start

### 1. Building the Project

#### Option A: Complete Local Build (on JetBot)
```bash
# Clone the repository
git clone <repository-url>
cd jetbot_ros2_mipt

# Build the application Docker image (includes base build)
docker build -t jetbot_full_ros2 .

# Start the container
docker-compose up -d
docker exec -it --privileged jetbot_ros2_mipt-jetbot-1 bash

# Build your packages (first time)
cd ~/ros2_ws
colcon build --symlink-install
```

#### Option B: Cross-compilation (Recommended)
**Step 1: Build base image on host machine (AMD64)**
```bash
# Navigate to docker configs
cd docker_configs

# Build base image and save as tar (for transfer to JetBot)
./build_for_amd.sh tar jetbot_ros2_base.tar

# Or build and push to Docker Hub
./build_for_amd.sh push v1.0.0
```

**Step 2: Transfer to JetBot and build application**
```bash
# Transfer tar to JetBot
scp jetbot_ros2_base.tar jetbot@<jetbot-ip>:~

# On JetBot: Load base image and build application
docker load < jetbot_ros2_base.tar
docker build -t jetbot_full_ros2 .
```

### 2. Building Your ROS2 Packages

#### Initial setup
```bash
# Start the container
docker-compose up -d

# Enter the container
docker exec -it --privileged jetbot_ros2_mipt-jetbot-1 bash
```

#### Build commands
```bash
# Build all packages
cd ~/ros2_ws
colcon build --symlink-install

# Build specific package
colcon build --packages-select diffdrive_jetbot

# Build with parallel jobs (adjust number based on your system)
colcon build --symlink-install --parallel-workers 2

# Source the workspace
source install/setup.bash
```

#### Development workflow
```bash
# After making changes to source code
colcon build --packages-select <package_name>
source install/setup.bash

# For fast iteration (only builds changed packages)
colcon build --symlink-install --packages-up-to <package_name>
```

## Usage

### Launch All Sensors and Drivers
Start all hardware drivers including motors, LIDAR, and joystick control:
```bash
ros2 launch jetbot_bringup activate_all_drivers.launch.py
```

This launches:
- Motor controllers (diffdrive_jetbot)
- RPLidar driver
- Joystick input handling
- Twist multiplexer for command prioritization

### SLAM Mapping
Create a map of the environment:
```bash
ros2 launch jetbot_bringup mapping.launch.py
```

This starts:
- slam_toolbox in online async mode
- Real-time map building from LIDAR data

### Navigation
Navigate using the created map:
```bash
ros2 launch jetbot_bringup nav2_bringup.launch.py
```

Launches the full Nav2 stack:
- Path planning
- Obstacle avoidance
- Goal-based navigation

### Localization (with existing map)
For localization against a pre-built map:
```bash
ros2 launch jetbot_bringup localization.launch.py
```

## General Scripts

### Docker Build Scripts
Located in `docker_configs/`:

- **`build_for_amd.sh`**: Cross-compilation script for building ARM64 base images on AMD64 machines
  ```bash
  # Save as tar file
  ./build_for_amd.sh tar jetbot_ros2_base.tar
  
  # Push to Docker registry
  ./build_for_amd.sh push v1.0.0
  ```

- **`find_ros_packages.py`**: Helper script to find ROS packages by prefix during base image build
- **`filter_rosinstall.py`**: Filters out already installed packages from rosinstall files to avoid conflicts

### Robot Control
- **Joystick Control**: Use connected gamepad for manual control
- **Twist Commands**: Send velocity commands via `/cmd_vel` topic
- **Multiple Input Sources**: Automatic switching between joystick, navigation, and manual control

## Architecture

### Packages

1. **`jetbot_bringup`**: Main launch package
   - Launch files for different operation modes
   - Configuration files for all components

2. **`diffdrive_jetbot`**: Hardware interface package
   - ROS2 Control hardware interface
   - Serial communication with motor controller
   - Wheel odometry and velocity control

### Hardware Communication
- **Motors**: Serial protocol via `/dev/ttyMOTOR` at 115200 baud
- **LIDAR**: RPLidar A1/A2 via `/dev/ttyLIDAR`
- **Camera**: USB camera for future vision applications

### Control Flow
```
Joystick/Nav2 → twist_mux → diff_drive_controller → Hardware Interface → Motors
                    ↑
LIDAR → slam_toolbox/nav2 → Local/Global Planning
```

## Docker Images Explained

### Base Docker Image (`docker_configs/Dockerfile_base`)
Contains general-purpose ROS2 components:
- ROS2 Humble core packages
- Navigation2 complete stack
- RQT tools suite and RViz2
- SLAM Toolbox
- ROS2 Control framework
- Build tools and dependencies

**Why build on host machine?**
- Faster compilation on powerful AMD64 hardware
- Reduced thermal load on JetBot
- Consistent build environment
- Can be reused across multiple JetBot projects

### Application Docker Image (`Dockerfile`)
Adds JetBot-specific packages:
- Additional teleop and control packages
- Joystick drivers
- Robot-specific utilities
- Your custom packages (src/ directory)

**Build considerations:**
- Lighter build process (fewer packages)
- Can be built on JetBot if needed
- Contains application-specific configurations

## Configuration

### Motor Parameters
Configure in URDF/launch files:
- `mtype`, `mline`, `mphase`: Motor encoder settings
- `pid_p`, `pid_i`, `pid_d`: PID control parameters
- `max_motor_rpm`: Maximum motor speed

### Navigation Tuning
Edit `config/nav2_default_params.yaml` for:
- Path planning parameters
- Obstacle avoidance settings
- Controller tuning

## Troubleshooting

### Common Issues
1. **Device permissions**: Ensure user has access to `/dev/ttyMOTOR` and `/dev/ttyLIDAR`
2. **ROS_DOMAIN_ID**: Set consistent domain ID across all nodes
3. **Transform errors**: Check that all required transforms are published
4. **Build failures**: Check available memory (use fewer parallel workers if needed)

### Debugging
```bash
# Check hardware interfaces
ros2 control list_hardware_interfaces

# Monitor controller status
ros2 control list_controllers

# View topics and transforms
ros2 topic list
ros2 run tf2_tools view_frames

# Check build logs
colcon build --event-handlers console_direct+
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Test on actual hardware
4. Submit a pull request

## License

[Add your license information here]

## Acknowledgments

- Based on ROS2 Control framework
- Uses Navigation2 for autonomous navigation
- SLAM implementation via slam_toolbox
