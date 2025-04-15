# Setup Instructions for Arduino Motor Bridge Package

This guide will help you resolve the "Could not find a package configuration file provided by ament_cmake" error and properly setup the ROS2 Arduino bridge.

## Prerequisites

Before building the package, ensure you have:

1. ROS2 properly installed and sourced
2. The necessary ROS2 development tools

## Step 1: Install Required ROS2 Dependencies

```bash
# Update package lists
sudo apt update

# Install ROS2 build dependencies
sudo apt install -y python3-colcon-common-extensions \
                    python3-rosdep \
                    python3-vcstool \
                    ros-$ROS_DISTRO-ament-cmake \
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

Add this to your `~/.bashrc` to source it automatically:

```bash
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 3: Create and Setup Workspace

```bash
# Create a new workspace if you don't already have one
mkdir -p ~/ros2_arduino_ws/src
cd ~/ros2_arduino_ws/src

# Copy the arduino_motor_bridge package to your workspace
cp -r /path/to/ros2_arduino_bridge/ros2_ws/src/arduino_motor_bridge .

# Install dependencies using rosdep
cd ~/ros2_arduino_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Step 4: Build the Package

```bash
# Build only the arduino_motor_bridge package
cd ~/ros2_arduino_ws
colcon build --packages-select arduino_motor_bridge

# Source the workspace
source ~/ros2_arduino_ws/install/setup.bash
```

## Step 5: Test the Package

After building successfully, try running the bridge:

```bash
# Make the Python scripts executable
chmod +x ~/ros2_arduino_ws/src/arduino_motor_bridge/scripts/arduino_bridge_node.py
chmod +x ~/ros2_arduino_ws/src/arduino_motor_bridge/scripts/test_arduino_commands.py

# Launch the Arduino bridge
ros2 launch arduino_motor_bridge arduino_bridge.launch.py
```

## Troubleshooting

If you still encounter the "ament_cmake not found" error:

1. Check that ROS2 is installed correctly:
   ```bash
   ros2 --version
   ```

2. Verify that ament_cmake is installed:
   ```bash
   apt list --installed | grep ament-cmake
   ```

3. If needed, install it manually:
   ```bash
   sudo apt install ros-$ROS_DISTRO-ament-cmake
   ```

4. Check your CMAKE_PREFIX_PATH:
   ```bash
   echo $CMAKE_PREFIX_PATH
   ```
   It should include your ROS2 installation path.

## Alternative: Python-only Package Approach

If you continue to have issues with the CMake-based approach, you can convert this to a Python-only package:

1. Replace the `CMakeLists.txt` with a simpler version focused on Python
2. Change `package.xml` to use `ament_python` instead of `ament_cmake`

Contact me if you need assistance with this alternative approach.
