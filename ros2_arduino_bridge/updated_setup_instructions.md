# Updated Setup Instructions for Arduino Motor Bridge Package

We've converted the package to a pure Python package using `ament_python` instead of `ament_cmake`. This should resolve build issues and make it easier to install on your Jetson Orin NX.

## Prerequisites

Before building the package, ensure you have:

1. ROS2 properly installed and sourced
2. The python3-serial package installed 

## Step 1: Install Required ROS2 Dependencies

```bash
# Update package lists
sudo apt update

# Install ROS2 build dependencies
sudo apt install -y python3-colcon-common-extensions \
                    python3-rosdep \
                    python3-serial

# Initialize rosdep if you haven't already
sudo rosdep init
rosdep update
```

## Step 2: Source ROS2 Environment

Make sure your ROS2 environment is properly sourced before building:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

## Step 3: Setup Your Existing Workspace

Since you already have a workspace at `/home/x4/ros2_ws_arduino`, you'll use that instead of creating a new one:

```bash
# Navigate to your workspace source directory
cd /home/x4/ros2_ws_arduino/src

# Copy the arduino_motor_bridge package to your workspace
cp -r /path/to/ros2_arduino_bridge/ros2_ws/src/arduino_motor_bridge .

# Install dependencies using rosdep
cd /home/x4/ros2_ws_arduino
rosdep install --from-paths src --ignore-src -r -y
```

## Step 4: Build the Python Package

```bash
# Build only the arduino_motor_bridge package
cd /home/x4/ros2_ws_arduino
colcon build --packages-select arduino_motor_bridge

# Source the workspace
source /home/x4/ros2_ws_arduino/install/setup.bash
```

## Step 5: Test the Package

Make sure the Arduino is connected to your Jetson via USB, then run:

```bash
# Launch the Arduino bridge
ros2 launch arduino_motor_bridge arduino_bridge.launch.py
```

In another terminal:

```bash
# Source your workspace
source /home/x4/ros2_ws_arduino/install/setup.bash

# Run the test sequence
ros2 run arduino_motor_bridge test_arduino_commands
```

## Troubleshooting

If you encounter any issues:

1. Check that the Arduino is properly connected and the correct serial port is specified
   ```bash
   # Check available serial ports
   ls /dev/ttyACM* /dev/ttyUSB*
   
   # Launch with specific port
   ros2 launch arduino_motor_bridge arduino_bridge.launch.py serial_port:=/dev/ttyACM0
   ```

2. Ensure the Arduino has the correct sketch uploaded:
   ```bash
   # Upload the sketch using Arduino IDE
   # Open the ros2_motor_controller.ino file in Arduino IDE and upload
   ```

3. Check that pyserial is properly installed:
   ```bash
   pip3 list | grep pyserial
   # If not installed:
   pip3 install pyserial
   ```

4. Check ROS2 environment:
   ```bash
   # Verify ROS2 is properly sourced
   ros2 topic list
   ```

## Package Structure

The package now follows the standard ROS2 Python package structure:

```
arduino_motor_bridge/
├── arduino_motor_bridge/          # Python package
│   ├── __init__.py
│   ├── arduino_bridge_node.py     # Main bridge node
│   ├── arduino_serial.py          # Serial communication library
│   └── test_arduino_commands.py   # Test script
├── config/                        # Configuration files
│   └── arduino_params.yaml
├── launch/                        # Launch files
│   └── arduino_bridge.launch.py
├── resource/                      # Package resources
│   └── arduino_motor_bridge
├── package.xml                    # Package manifest
└── setup.py                       # Python setup script
```

## Using the Package in ROS2 Navigation

This package can be integrated into a full ROS2 navigation stack. The bridge node publishes odometry and listens for velocity commands, which can be used by the navigation stack:

```bash
# Example: Manually send a velocity command
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Reset odometry if needed
ros2 service call /reset_odom std_srvs/srv/Trigger
```
