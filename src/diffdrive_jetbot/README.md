# diffdrive_jetbot

ROS 2 hardware interface for Jetbot differential drive with multi-robot support.

## Features
- Compatible with `diff_drive_controller` from `ros2_control`
- Serial communication over two motors with velocity control and feedback
- Multi-robot namespace support (up to 5 robots)
- Individual tf frames for each robot

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

### Single Robot
```bash
# Bring up hardware + controllers
ros2 launch diffdrive_jetbot diffbot.launch.py
```

### Multi-Robot System
Each robot should be launched with a unique robot_id (1-5):

```bash
# On Robot 1
ros2 launch diffdrive_jetbot diffbot.launch.py robot_id:=1

# On Robot 2  
ros2 launch diffdrive_jetbot diffbot.launch.py robot_id:=2

# etc...
```

### Complete System Launch
```bash
# Launch all drivers for a specific robot
ros2 launch jetbot_bringup activate_all_drivers.launch.py robot_id:=1
```

## Multi-Robot Architecture

### Namespace Structure
- Robot 1: `/robot_1/`
- Robot 2: `/robot_2/`
- Robot 3: `/robot_3/`
- Robot 4: `/robot_4/`
- Robot 5: `/robot_5/`

### TF Frame Structure
- Robot 1: `robot_1_base_link`, `robot_1_left_wheel_link`, etc.
- Robot 2: `robot_2_base_link`, `robot_2_left_wheel_link`, etc.
- etc...

### Topics
Each robot publishes/subscribes to topics within its namespace:
- `/robot_X/cmd_vel` - Velocity commands
- `/robot_X/joint_states` - Joint state information  
- `/robot_X/odom` - Odometry data
- `/robot_X/scan` - Lidar data (if equipped)
