# Fixing the Executable Issue in arduino_motor_bridge Package

The error `libexec directory does not exist` typically happens when ROS2 can't find the executable scripts. This is a common issue with Python-based packages. Let's fix it:

## Step 1: Check your entry points in setup.py

Make sure your setup.py has the correct entry points that match the Python module structure:

```python
entry_points={
    'console_scripts': [
        'arduino_bridge_node = arduino_motor_bridge.arduino_bridge_node:main',
        'test_arduino_commands = arduino_motor_bridge.test_arduino_commands:main',
    ],
},
```

## Step 2: Move the scripts to the right location

Since we've set up our package as a pure Python package, the scripts need to be in the Python module directory, not in a separate scripts directory:

```bash
# Navigate to the package source
cd /home/x4/ros2_ws_arduino/src/arduino_motor_bridge

# Ensure permissions are correct on the Python files
chmod +x arduino_motor_bridge/arduino_bridge_node.py
chmod +x arduino_motor_bridge/test_arduino_commands.py 
```

## Step 3: Update the launch file

The launch file needs to use the correct executable name (without the .py extension):

Edit the launch file to use:
```python
arduino_bridge_node = Node(
    package='arduino_motor_bridge',
    executable='arduino_bridge_node',  # Not arduino_bridge_node.py
    name='arduino_bridge',
    output='screen',
    parameters=[{...}]
)
```

## Step 4: Clean build and reinstall

```bash
# Navigate to your workspace
cd /home/x4/ros2_ws_arduino

# Clean build files
rm -rf build/arduino_motor_bridge install/arduino_motor_bridge

# Rebuild
colcon build --packages-select arduino_motor_bridge

# Source the workspace
source install/setup.bash
```

## Step 5: Test Again

```bash
# Try launching again
ros2 launch arduino_motor_bridge arduino_bridge.launch.py
```

## Alternative Solution: Direct Script Execution

If you're still having issues with the launch file, you can try running the Python script directly for testing:

```bash
# Navigate to your package directory
cd /home/x4/ros2_ws_arduino/src/arduino_motor_bridge

# Run the script directly
python3 arduino_motor_bridge/arduino_bridge_node.py
```
