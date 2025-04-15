# Arduino Tracked Robot Controller

A comprehensive control system for dual-track robot platforms using Arduino. This system provides precise motor control, track synchronization, and diagnostic capabilities for tracked robot platforms.

## Hardware Requirements

- Arduino board (Uno, Mega, or similar)
- Two DC motor controllers/drivers
- Two DC motors with Hall effect sensors (for tracked wheels)
- Power supply adequate for motors (typically 12V or higher)
- Breadboard and jumper wires for connections

## Wiring Diagram

### Motor Controller Connections (For Each Controller)

#### Left Motor Controller:
- Connect Arduino pin 9 (leftThrottlePin) → Throttle wire (#7 in controller)
- Connect Arduino pin 7 (leftDirPin) → Reverse line (#3 in controller)
- Connect Arduino 5V → Throttle power wire (usually red)
- Connect Arduino GND → Throttle ground wire (usually black)
- Connect the 3 hall sensor wires from left motor → Arduino pins 2, 3, 4
- Connect battery negative → Controller battery negative (#13 black wire)
- Connect battery positive → Controller battery positive (#13 red wire)

#### Right Motor Controller:
- Connect Arduino pin 10 (rightThrottlePin) → Throttle wire (#7 in controller)
- Connect Arduino pin 8 (rightDirPin) → Reverse line (#3 in controller)
- Connect Arduino 5V → Throttle power wire (usually red)
- Connect Arduino GND → Throttle ground wire (usually black)
- Connect the 3 hall sensor wires from right motor → Arduino pins 5, 6, 12
- Connect battery negative → Controller battery negative (#13 black wire)
- Connect battery positive → Controller battery positive (#13 red wire)

### Power Supply
- Use a separate power supply for the Arduino (USB or dedicated supply)
- Use a powerful 48V battery for the motors (connected to both controllers)
- Make sure to have common ground between Arduino and motor controllers

## Installation

1. Connect all hardware according to the wiring diagram above
2. Install Arduino IDE (if not already installed)
3. Connect Arduino to computer via USB
4. Open Arduino IDE and load the `motor_controller.ino` file
5. Select your Arduino board type and port
6. Click Upload to flash the code to your Arduino

## Usage Guide

### Main Control Commands

| Key | Function | Description |
|-----|----------|-------------|
| w | Forward | Move both tracks forward |
| s | Backward | Move both tracks backward |
| a | Turn Left | Slow left track for gradual left turn |
| d | Turn Right | Slow right track for gradual right turn |
| q | Pivot Left | Left track backward, right track forward for spot turn |
| e | Pivot Right | Left track forward, right track backward for spot turn |
| x | Stop | Stop all movement |
| + | Increase Speed | Increase speed by 10 (0-255) |
| - | Decrease Speed | Decrease speed by 10 (0-255) |
| c | Synchronize | Synchronize tracks if they've become misaligned |
| ? | Menu | Display command menu |
| # | Debug | Show debugging information |

### Testing and Diagnostic Commands

| Key | Function | Description |
|-----|----------|-------------|
| 1 | Test Left Forward | Test left motor in forward direction |
| 2 | Test Left Backward | Test left motor in backward direction |
| 3 | Test Right Forward | Test right motor in forward direction |
| 4 | Test Right Backward | Test right motor in backward direction |
| 5 | Toggle Left Dir | Toggle direction pin for left motor |
| 6 | Toggle Right Dir | Toggle direction pin for right motor |
| 7 | Special Backward Test | Run detailed backward motion test sequence |

## Key Features

1. **Smooth Direction Changes**: System properly handles direction changes with appropriate timing for motor controllers
2. **Speed Control Without Stopping**: Change speed on-the-fly without interrupting movement
3. **Track Synchronization**: Automatically compensate for track misalignment
4. **Diagnostic Tools**: Comprehensive testing and debugging capabilities
5. **Position Tracking**: Hall effect sensor integration for position tracking

## Working Principles

- **Direction Control**: The direction pins (7 and 8) control motor direction (LOW = forward, HIGH = backward)
- **Speed Control**: PWM signals on throttle pins (9 and 10) control motor speed (0-255)
- **Position Tracking**: Hall effect sensors track wheel rotation for synchronization
- **Turns**: Differential speed control for regular turns, opposite directions for pivot turns

## Troubleshooting

### Common Issues

1. **Motors Don't Move**
   - Check all power connections
   - Verify motor controller wiring
   - Check that throttle signals are reaching controllers
   - Test individual motors with test commands (1-4)

2. **Direction Issues**
   - Use debug command (#) to verify direction pin states
   - Check that direction signals are reaching controllers
   - Test direction pin toggle commands (5-6)

3. **Speed Control Problems**
   - Ensure PWM signals are reaching throttle inputs
   - Verify power supply is adequate for desired speed
   - Test with special backward test (7) to see progressive speed changes

4. **Track Synchronization Problems**
   - Verify hall sensors are working properly
   - Check that synchronization function is being triggered
   - Ensure both motors are mechanically sound

## Customization

You can modify these parameters in the code to adjust performance:

- `baseSpeed`: Default movement speed (0-255)
- `turnDifference`: Speed reduction for turning (higher = sharper turns)
- `currentSpeed`: Current operating speed (modified by + and - commands)
- `resetMotors()` delay values: Timing for direction changes (adjust if motors stutter)

## Notes

- The Arduino Serial Monitor must be set to 115200 baud to communicate with the control system
- For safety, always start at low speeds when testing
- The controllers require a brief delay after direction changes before applying throttle
- Hall sensors should increment position regardless of direction for basic tracking

## License

This project is provided as open-source software. Feel free to modify and distribute as needed.
