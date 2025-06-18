# diffdrive_jetbot

ROS 2 hardware interface for Jetbot differential drive.

## Features
- Compatible with `diff_drive_controller` from `ros2_control`
- Serial communication over two motors with velocity control and feedback

## Installation
```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone https://github.com/your_user/diffdrive_jetbot.git
# Build the package
cd ~/ros2_ws
colcon build --packages-select diffdrive_jetbot
```

## Usage
```bash
# Bring up hardware + controllers + RViz
ros2 launch diffdrive_jetbot diffbot.launch.py
```
