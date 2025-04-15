# ROS2 Arduino Motor Controller Bridge

This package provides a bridge between ROS2 running on Jetson Orin NX and Arduino-based motor controllers. It allows the ROS2 navigation stack to control motors while the Arduino handles the low-level motor control and provides odometry feedback.

## Overview

The system consists of two main parts:

1. **Arduino Controller**: Handles the real-time control of the motors, reads hall effect sensors, and provides odometry data.
2. **ROS2 Bridge**: Converts ROS2 commands to Arduino protocol and publishes Arduino sensor data back to ROS2.

## Prerequisites

- Arduino IDE
- ROS2 Humble (or later) installed on Jetson Orin NX
- Python 3.8+ with pyserial package
- USB connection between Jetson and Arduino

## Hardware Setup

This system is designed for a differential drive robot with:
- Two motors with hall effect sensors
- Two motor controllers (one for each motor)
- Arduino (Uno, Mega, or compatible board)
- Jetson Orin NX for ROS2 navigation and control

Refer to the wiring diagram in the Arduino sketch comments for detailed connection information.

## Installation

### Arduino Setup

1. Open the Arduino IDE and load the `ros2_motor_controller.ino` sketch from the `arduino` folder
2. Configure the pin settings in the sketch to match your hardware
3. Upload the sketch to your Arduino

### ROS2 Package Setup

1. Copy the `arduino_motor_bridge` package to your ROS2 workspace:
   ```bash
   cp -r ros2_arduino_bridge/ros2_ws/src/arduino_motor_bridge ~/ros2_ws/src/
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   sudo apt install python3-serial
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select arduino_motor_bridge
   source install/setup.bash
   ```

## Usage

### Basic ROS2 Bridge Usage

1. Connect the Arduino to the Jetson via USB
2. Launch the ROS2 bridge:
   ```bash
   ros2 launch arduino_motor_bridge arduino_bridge.launch.py
   ```

3. Send velocity commands:
   ```bash
   ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

### Configuration

The bridge behavior can be configured using parameters in:
- Launch file arguments
- YAML configuration file in the `config` directory
- Direct parameter setting via ROS2 parameter services

Key parameters:
- `serial_port`: Arduino serial port (default: /dev/ttyACM0)
- `baud_rate`: Communication speed (default: 115200)
- `wheel_separation`: Distance between wheels in meters
- `wheel_radius`: Wheel radius in meters

### Test Script

A test script is included to exercise the robot's movement capabilities:

```bash
ros2 run arduino_motor_bridge test_arduino_commands.py
```

This script will run the robot through a sequence of movements (forward, turn, backward, etc.) to verify functionality.

## ROS2 Integration

### Published Topics

- `/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)): Odometry information from wheel encoders
- `/tf` (if enabled): Transform from `odom` frame to `base_link`

### Subscribed Topics

- `/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)): Velocity commands

### Services

- `/reset_odom` ([std_srvs/Trigger](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html)): Resets odometry to zero

## Troubleshooting

### Serial Connection Issues

- Verify your Arduino is connected and the port is correct
- Check permissions: `sudo usermod -a -G dialout $USER` (logout/login required)
- Verify baudrate matches Arduino sketch settings

### Motor Control Issues

- Run the Arduino test commands directly through serial monitor
- Use the debug command (#) to check pin states
- Verify motor controller wiring matches the pin definitions

### Odometry Issues

- Verify hall sensor wiring and connections
- Check the wheel_radius and wheel_separation parameters
- Manually test the hall sensors with test functions

## License

This package is provided under the Apache 2.0 license.
