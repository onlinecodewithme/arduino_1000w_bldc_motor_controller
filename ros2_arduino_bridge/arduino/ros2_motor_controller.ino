#include <Arduino.h>

// Pin definitions
const int leftThrottlePin = 9;  // PWM output to left throttle
const int rightThrottlePin = 10; // PWM output to right throttle
const int leftDirPin = 7;       // Left direction control
const int rightDirPin = 8;      // Right direction control
const int leftHallPins[] = {2, 3, 4};   // Left motor hall sensors
const int rightHallPins[] = {5, 6, 12}; // Right motor hall sensors

// Position tracking
volatile long leftPosition = 0;
volatile long rightPosition = 0;
unsigned long lastOdomTime = 0;
const unsigned long ODOM_INTERVAL = 100; // Publish odometry at 10Hz

// Robot physical parameters
const float WHEEL_DIAMETER = 0.165;  // in meters - adjust to your robot's wheel size
const float WHEEL_SEPARATION = 0.35; // in meters - adjust to your robot's track width
const float TICKS_PER_REVOLUTION = 360.0; // Adjust to your encoder resolution
const float METERS_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REVOLUTION;

// Movement parameters
int baseSpeed = 80;      // Base movement speed (0-255)
int turnDifference = 50;  // Speed difference for turning
bool moving = false;      // Movement state
int currentSpeed = 150;   // Current speed value (0-255)

// ROS Command variables
char command = 0;
float targetLeftVel = 0.0;  // Target left wheel velocity in m/s
float targetRightVel = 0.0; // Target right wheel velocity in m/s

// Communication protocol constants
const char COMMAND_HEADER = '#';
const char VELOCITY_HEADER = 'V';
const char ODOMETRY_HEADER = 'O';
const char RESET_HEADER = 'R';
const char ACK_HEADER = 'A';
const char DEBUG_HEADER = 'D';

void setup() {
  // Setup pins
  pinMode(leftThrottlePin, OUTPUT);
  pinMode(rightThrottlePin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  
  // Initialize hall sensor pins and attach interrupts
  for (int i = 0; i < 3; i++) {
    pinMode(leftHallPins[i], INPUT_PULLUP);
    pinMode(rightHallPins[i], INPUT_PULLUP);
  }
  
  // Attach interrupts for position tracking
  attachInterrupt(digitalPinToInterrupt(leftHallPins[0]), leftHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightHallPins[0]), rightHallChange, CHANGE);
  
  // Initial state - stopped
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  digitalWrite(leftDirPin, LOW);   // Forward
  digitalWrite(rightDirPin, LOW);  // Forward
  
  Serial.begin(115200);
  delay(100);
  Serial.println("ROS2 Arduino Motor Controller Initialized");
}

void loop() {
  // Process any incoming commands
  processSerialInput();
  
  // Send odometry data on the specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastOdomTime >= ODOM_INTERVAL) {
    sendOdometryData();
    lastOdomTime = currentTime;
  }
  
  // Update motor speeds based on target velocities
  updateMotorSpeeds();
}

void processSerialInput() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char header = Serial.read();
    
    if (header == COMMAND_HEADER) {
      // Manual command mode (for testing)
      if (Serial.available() > 0) {
        command = Serial.read();
        processManualCommand(command);
      }
    }
    else if (header == VELOCITY_HEADER) {
      // Velocity command from ROS2
      if (Serial.available() >= 8) { // 2 float values (4 bytes each)
        float leftVel, rightVel;
        
        // Read 4 bytes for leftVel
        byte leftBytes[4];
        for (int i = 0; i < 4; i++) {
          leftBytes[i] = Serial.read();
        }
        
        // Read 4 bytes for rightVel
        byte rightBytes[4];
        for (int i = 0; i < 4; i++) {
          rightBytes[i] = Serial.read();
        }
        
        // Convert bytes to floats
        memcpy(&leftVel, leftBytes, 4);
        memcpy(&rightVel, rightBytes, 4);
        
        // Set target velocities
        targetLeftVel = leftVel;
        targetRightVel = rightVel;
        
        // Acknowledge the command
        Serial.print(ACK_HEADER);
        Serial.println("VEL_CMD");
      }
    }
    else if (header == RESET_HEADER) {
      // Reset position counters
      resetPositionCounters();
      Serial.print(ACK_HEADER);
      Serial.println("RESET");
    }
    else if (header == DEBUG_HEADER) {
      // Send debug information
      sendDebugInfo();
    }
  }
}

void processManualCommand(char cmd) {
  switch(cmd) {
    case 'w': // Forward
      Serial.println("Moving forward");
      moveForward(currentSpeed);
      break;
    case 's': // Backward
      Serial.println("Moving backward");
      moveBackward(currentSpeed);
      break;
    case 'a': // Left turn
      Serial.println("Turning left");
      turnLeft(currentSpeed);
      break;
    case 'd': // Right turn
      Serial.println("Turning right");
      turnRight(currentSpeed);
      break;
    case 'q': // Pivot left
      Serial.println("Pivoting left");
      pivotLeft(currentSpeed);
      break;
    case 'e': // Pivot right
      Serial.println("Pivoting right");
      pivotRight(currentSpeed);
      break;
    case 'x': // Stop
      Serial.println("Stopping");
      stop();
      break;
    case '+': // Increase speed
      currentSpeed = min(255, currentSpeed + 10);
      Serial.print("Speed increased to: ");
      Serial.println(currentSpeed);
      updateSpeedOnly();
      break;
    case '-': // Decrease speed
      currentSpeed = max(0, currentSpeed - 10);
      Serial.print("Speed decreased to: ");
      Serial.println(currentSpeed);
      updateSpeedOnly();
      break;
    case '?': // Print debug info
      sendDebugInfo();
      break;
    default:
      // Ignore other characters
      break;
  }
}

void resetPositionCounters() {
  // Reset position counters to zero
  noInterrupts(); // Disable interrupts while updating volatile variables
  leftPosition = 0;
  rightPosition = 0;
  interrupts(); // Re-enable interrupts
}

void sendOdometryData() {
  // Get current positions
  noInterrupts(); // Disable interrupts while reading volatile variables
  long currentLeftPos = leftPosition;
  long currentRightPos = rightPosition;
  interrupts(); // Re-enable interrupts
  
  // Calculate distance traveled in meters
  float leftDist = currentLeftPos * METERS_PER_TICK;
  float rightDist = currentRightPos * METERS_PER_TICK;
  
  // Send odometry data in format: O,leftPos,rightPos,leftDist,rightDist
  Serial.print(ODOMETRY_HEADER);
  Serial.print(",");
  Serial.print(currentLeftPos);
  Serial.print(",");
  Serial.print(currentRightPos);
  Serial.print(",");
  Serial.print(leftDist, 6);  // 6 decimal places
  Serial.print(",");
  Serial.println(rightDist, 6);  // 6 decimal places
}

void sendDebugInfo() {
  Serial.print(DEBUG_HEADER);
  Serial.print(",LeftPos:");
  Serial.print(leftPosition);
  Serial.print(",RightPos:");
  Serial.print(rightPosition);
  Serial.print(",LeftDir:");
  Serial.print(digitalRead(leftDirPin));
  Serial.print(",RightDir:");
  Serial.print(digitalRead(rightDirPin));
  Serial.print(",LeftThrottle:");
  Serial.print(analogRead(leftThrottlePin));
  Serial.print(",RightThrottle:");
  Serial.print(analogRead(rightThrottlePin));
  Serial.print(",TargetLeftVel:");
  Serial.print(targetLeftVel);
  Serial.print(",TargetRightVel:");
  Serial.println(targetRightVel);
}

void updateMotorSpeeds() {
  // Map velocity targets to motor PWM values
  // Note: Maximum speed will depend on your specific motors
  const float MAX_SPEED_MPS = 1.0; // Maximum speed in meters per second
  const int MAX_PWM = 255;         // Maximum PWM value
  
  // Calculate left motor PWM and direction
  int leftPWM = abs(targetLeftVel) * (MAX_PWM / MAX_SPEED_MPS);
  leftPWM = constrain(leftPWM, 0, MAX_PWM);
  boolean leftDir = (targetLeftVel >= 0) ? LOW : HIGH; // LOW = forward, HIGH = backward
  
  // Calculate right motor PWM and direction
  int rightPWM = abs(targetRightVel) * (MAX_PWM / MAX_SPEED_MPS);
  rightPWM = constrain(rightPWM, 0, MAX_PWM);
  boolean rightDir = (targetRightVel >= 0) ? LOW : HIGH; // LOW = forward, HIGH = backward
  
  // Set motor directions first if they've changed
  if (digitalRead(leftDirPin) != leftDir || digitalRead(rightDirPin) != rightDir) {
    // When changing directions, first stop the motors
    analogWrite(leftThrottlePin, 0);
    analogWrite(rightThrottlePin, 0);
    delay(50); // Small delay to let motors stop completely
    
    // Set new directions
    digitalWrite(leftDirPin, leftDir);
    digitalWrite(rightDirPin, rightDir);
    delay(5); // Small delay to ensure direction change takes effect
  }
  
  // Apply throttle
  analogWrite(leftThrottlePin, leftPWM);
  analogWrite(rightThrottlePin, rightPWM);
  
  // Update moving state
  moving = (leftPWM > 0 || rightPWM > 0);
}

// Functions for manual control, same as before but with some simplifications

void updateSpeedOnly() {
  int leftSpeed, rightSpeed;
  
  switch(command) {
    case 'w': // Forward
      analogWrite(leftThrottlePin, currentSpeed);
      analogWrite(rightThrottlePin, currentSpeed);
      break;
    case 's': // Backward
      analogWrite(leftThrottlePin, currentSpeed);
      analogWrite(rightThrottlePin, currentSpeed);
      break;
    case 'a': // Left turn
      leftSpeed = max(0, currentSpeed - turnDifference);
      analogWrite(leftThrottlePin, leftSpeed);
      analogWrite(rightThrottlePin, currentSpeed);
      break;
    case 'd': // Right turn
      rightSpeed = max(0, currentSpeed - turnDifference);
      analogWrite(leftThrottlePin, currentSpeed);
      analogWrite(rightThrottlePin, rightSpeed);
      break;
    case 'q': // Pivot left
    case 'e': // Pivot right
      analogWrite(leftThrottlePin, currentSpeed);
      analogWrite(rightThrottlePin, currentSpeed);
      break;
  }
}

void resetMotors() {
  // First stop all motors completely
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  delay(50); // Small delay to let motors stop completely
}

void moveForward(int speed) {
  resetMotors();
  digitalWrite(leftDirPin, LOW);
  digitalWrite(rightDirPin, LOW);
  delay(5);
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
  command = 'w';
}

void moveBackward(int speed) {
  resetMotors();
  digitalWrite(leftDirPin, HIGH);
  digitalWrite(rightDirPin, HIGH);
  delay(5);
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
  command = 's';
}

void turnLeft(int speed) {
  resetMotors();
  digitalWrite(leftDirPin, LOW);
  digitalWrite(rightDirPin, LOW);
  int leftSpeed = max(0, speed - turnDifference);
  analogWrite(leftThrottlePin, leftSpeed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
  command = 'a';
}

void turnRight(int speed) {
  resetMotors();
  digitalWrite(leftDirPin, LOW);
  digitalWrite(rightDirPin, LOW);
  int rightSpeed = max(0, speed - turnDifference);
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, rightSpeed);
  moving = true;
  command = 'd';
}

void pivotLeft(int speed) {
  resetMotors();
  digitalWrite(leftDirPin, HIGH);
  digitalWrite(rightDirPin, LOW);
  delay(5);
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
  command = 'q';
}

void pivotRight(int speed) {
  resetMotors();
  digitalWrite(leftDirPin, LOW);
  digitalWrite(rightDirPin, HIGH);
  delay(5);
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
  command = 'e';
}

void stop() {
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  moving = false;
  command = 'x';
}

// Interrupt handlers for position tracking
void leftHallChange() {
  // Determine direction based on the direction pin state
  if (digitalRead(leftDirPin) == LOW) {
    leftPosition++; // Forward
  } else {
    leftPosition--; // Backward
  }
}

void rightHallChange() {
  // Determine direction based on the direction pin state
  if (digitalRead(rightDirPin) == LOW) {
    rightPosition++; // Forward
  } else {
    rightPosition--; // Backward
  }
}
