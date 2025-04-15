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

// Movement parameters
int baseSpeed = 80;      // Base movement speed (0-255)
int turnDifference = 50;  // Speed difference for turning
bool moving = false;      // Movement state
int currentSpeed = 150;   // Current speed value (0-255)

// Command variables
char command = 0;
char lastCommand = 0;  // Track last command for updateMovement

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
  Serial.println("Tracked Robot Control System Initialized");
  printMenu();
}

void loop() {
  // Check for keyboard commands
  if (Serial.available() > 0) {
    command = Serial.read();
    processCommand(command);
  }
  
  // Uncomment to constantly debug motor signals (might flood serial)
  // debugMotorSignals();
  // delay(500);
}

void printMenu() {
  Serial.println("\n=== Tracked Robot Control System ===");
  Serial.println("Controls:");
  Serial.println("  w - Move forward");
  Serial.println("  s - Move backward");
  Serial.println("  a - Turn left");
  Serial.println("  d - Turn right");
  Serial.println("  q - Pivot left");
  Serial.println("  e - Pivot right");
  Serial.println("  x - Stop");
  Serial.println("  + - Increase speed");
  Serial.println("  - - Decrease speed");
  Serial.println("  c - Synchronize tracks");
  Serial.println("  ? - Show this menu");
  Serial.println("  # - Debug motor signals");
  Serial.println("Test Controls:");
  Serial.println("  1 - Test left motor forward");
  Serial.println("  2 - Test left motor backward");
  Serial.println("  3 - Test right motor forward");
  Serial.println("  4 - Test right motor backward");
  Serial.println("Direction Pin Tests:");
  Serial.println("  5 - Toggle LEFT direction pin");
  Serial.println("  6 - Toggle RIGHT direction pin");
  Serial.println("  7 - Run special backward test");
  Serial.print("Current speed: ");
  Serial.println(currentSpeed);
}

void processCommand(char cmd) {
  switch(cmd) {
    case 'w': // Forward
      Serial.println("Moving forward");
      moveForward(currentSpeed);
      lastCommand = 'w';
      break;
    case 's': // Backward
      Serial.println("Moving backward");
      moveBackward(currentSpeed);
      lastCommand = 's';
      break;
    case 'a': // Left turn
      Serial.println("Turning left");
      turnLeft(currentSpeed);
      lastCommand = 'a';
      break;
    case 'd': // Right turn
      Serial.println("Turning right");
      turnRight(currentSpeed);
      lastCommand = 'd';
      break;
    case 'q': // Pivot left
      Serial.println("Pivoting left");
      pivotLeft(currentSpeed);
      lastCommand = 'q';
      break;
    case 'e': // Pivot right
      Serial.println("Pivoting right");
      pivotRight(currentSpeed);
      lastCommand = 'e';
      break;
    case 'x': // Stop
      Serial.println("Stopping");
      stop();
      lastCommand = 'x';
      break;
    case '+': // Increase speed
      currentSpeed = min(255, currentSpeed + 10);
      Serial.print("Speed increased to: ");
      Serial.println(currentSpeed);
      updateSpeedOnly();  // New function that doesn't reset motors
      break;
    case '-': // Decrease speed
      currentSpeed = max(0, currentSpeed - 10);
      Serial.print("Speed decreased to: ");
      Serial.println(currentSpeed);
      updateSpeedOnly();  // New function that doesn't reset motors
      break;
    case 'c': // Synchronize tracks
      Serial.println("Synchronizing tracks");
      synchronizeTracks();
      break;
    case '?': // Print menu
      printMenu();
      break;
    case '#': // Debug motor signals
      Serial.println("Debug Motor Signals:");
      debugMotorSignals();
      break;
    // Test commands
    case '1': // Test left motor forward
      Serial.println("Testing left motor forward");
      testLeftMotorForward();
      break;
    case '2': // Test left motor backward
      Serial.println("Testing left motor backward");
      testLeftMotorBackward();
      break;
    case '3': // Test right motor forward
      Serial.println("Testing right motor forward");
      testRightMotorForward();
      break;
    case '4': // Test right motor backward
      Serial.println("Testing right motor backward");
      testRightMotorBackward();
      break;
    case '5': // Toggle LEFT direction pin
      toggleLeftDirPin();
      break;
    case '6': // Toggle RIGHT direction pin
      toggleRightDirPin();
      break;
    case '7': // Special backward test
      specialBackwardTest();
      break;
    default:
      // Ignore other characters (like line feeds and carriage returns)
      if (cmd >= 32) { // Only print for visible characters
        Serial.print("Unknown command: ");
        Serial.println(cmd);
      }
      break;
  }
}

// New function for updating speed without stopping motors
void updateSpeedOnly() {
  if (!moving) return;
  
  int leftSpeed, rightSpeed;
  
  // Apply new speed based on current movement type
  switch(lastCommand) {
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

// Old update movement function preserved, but use new function for speed changes
void updateMovement() {
  // Re-apply the current movement with the new speed - use full movement command
  if (moving) {
    // Use the last command to determine the current movement
    switch(lastCommand) {
      case 'w':
        moveForward(currentSpeed);
        break;
      case 's':
        moveBackward(currentSpeed);
        break;
      case 'a':
        turnLeft(currentSpeed);
        break;
      case 'd':
        turnRight(currentSpeed);
        break;
      case 'q':
        pivotLeft(currentSpeed);
        break;
      case 'e':
        pivotRight(currentSpeed);
        break;
      default:
        // If no valid last command, do nothing
        break;
    }
  }
}

// Reset function - call before changing directions
void resetMotors() {
  // First stop all motors completely
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  delay(500); // Small delay to let motors stop completely
  
  // Don't change the moving state - this was causing the need for double commands
  
  // Reduce verbosity - this print was causing confusion
  // Serial.println("Motors reset");
}

// Basic movement functions
void moveForward(int speed) {
  // First reset motors to ensure clean direction change
  resetMotors();
  
  // Set direction pins for forward
  digitalWrite(leftDirPin, LOW);
  digitalWrite(rightDirPin, LOW);
  
  // Small delay to ensure direction change takes effect
  delay(500);
  
  // Print debug info
  Serial.println("FORWARD DEBUG INFO:");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin));
  
  // Apply throttle
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  
  moving = true;
}

void moveBackward(int speed) {
  // Use the reset function to ensure clean direction change
  resetMotors();
  
  // Force direction pins HIGH for reverse
  digitalWrite(leftDirPin, HIGH);
  digitalWrite(rightDirPin, HIGH);
  
  // Small delay to ensure direction change takes effect
  delay(500);
  
  // Print debug info
  Serial.println("REVERSE DEBUG INFO:");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin));
  
  // Now apply throttle
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  
  moving = true;
}

void turnLeft(int speed) {
  // Use the reset function to ensure clean direction change
  resetMotors();
  
  // Set directions for left turn
  digitalWrite(leftDirPin, LOW);    // Left track slower or stopped
  digitalWrite(rightDirPin, LOW);   // Right track forward
  
  // Print debug info
  Serial.println("TURN LEFT DEBUG INFO:");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin));
  
  // Apply throttle with different speeds
  int leftSpeed = max(0, speed - turnDifference);
  analogWrite(leftThrottlePin, leftSpeed);
  analogWrite(rightThrottlePin, speed);
  
  moving = true;
}

void turnRight(int speed) {
  // Use the reset function to ensure clean direction change
  resetMotors();
  
  // Set directions for right turn
  digitalWrite(leftDirPin, LOW);    // Left track forward
  digitalWrite(rightDirPin, LOW);   // Right track slower or stopped
  
  // Print debug info
  Serial.println("TURN RIGHT DEBUG INFO:");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin));
  
  // Apply throttle with different speeds
  analogWrite(leftThrottlePin, speed);
  int rightSpeed = max(0, speed - turnDifference);
  analogWrite(rightThrottlePin, rightSpeed);
  
  moving = true;
}

void pivotLeft(int speed) {
  // Use the reset function to ensure clean direction change
  resetMotors();
  
  // Set direction pins
  digitalWrite(leftDirPin, HIGH);   // Left track backward
  digitalWrite(rightDirPin, LOW);   // Right track forward
  
  // Small delay to ensure direction change takes effect
  delay(5);
  
  // Print debug info
  Serial.println("PIVOT LEFT DEBUG INFO:");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin));
  
  // Apply throttle
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void pivotRight(int speed) {
  // Use the reset function to ensure clean direction change
  resetMotors();
  
  // Set direction pins
  digitalWrite(leftDirPin, LOW);    // Left track forward
  digitalWrite(rightDirPin, HIGH);  // Right track backward
  
  // Small delay to ensure direction change takes effect
  delay(5);
  
  // Print debug info
  Serial.println("PIVOT RIGHT DEBUG INFO:");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin));
  
  // Apply throttle
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void stop() {
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  moving = false;
}

// Track synchronization function - ensures both tracks are aligned
void synchronizeTracks() {
  // Calculate position difference
  long posDiff = leftPosition - rightPosition;
  
  if (abs(posDiff) > 10) {  // Threshold for correction
    Serial.print("Position difference detected: ");
    Serial.println(posDiff);
    
    // Determine which track is ahead
    if (posDiff > 0) {
      // Left track ahead, move right track to catch up
      digitalWrite(leftDirPin, LOW);    // Hold left track
      digitalWrite(rightDirPin, LOW);   // Move right track forward
      analogWrite(leftThrottlePin, 0);
      analogWrite(rightThrottlePin, 100);
      
      // Wait until synchronized
      while (leftPosition - rightPosition > 5) {
        delay(10);
        // Check for incoming commands to allow interruption
        if (Serial.available() > 0) {
          char cmd = Serial.read();
          if (cmd == 'x') {
            Serial.println("Synchronization interrupted");
            break;
          }
        }
      }
    } else {
      // Right track ahead, move left track to catch up
      digitalWrite(leftDirPin, LOW);    // Move left track forward
      digitalWrite(rightDirPin, LOW);   // Hold right track
      analogWrite(leftThrottlePin, 100);
      analogWrite(rightThrottlePin, 0);
      
      // Wait until synchronized
      while (rightPosition - leftPosition > 5) {
        delay(10);
        // Check for incoming commands to allow interruption
        if (Serial.available() > 0) {
          char cmd = Serial.read();
          if (cmd == 'x') {
            Serial.println("Synchronization interrupted");
            break;
          }
        }
      }
    }
    
    // Stop both tracks after synchronization
    stop();
    Serial.println("Tracks synchronized!");
  } else {
    Serial.println("Tracks already in sync.");
  }
}

// Interrupt handlers for position tracking with debug output
void leftHallChange() {
  // Simple increment regardless of direction for basic tracking
  // Direction will be properly handled by the motor controller
  leftPosition++;
  
  // Debug output would go here if needed:
  // Serial.print("L:");
  // Serial.println(leftPosition);
}

void rightHallChange() {
  // Simple increment regardless of direction for basic tracking
  // Direction will be properly handled by the motor controller
  rightPosition++;
  
  // Debug output would go here if needed:
  // Serial.print("R:");
  // Serial.println(rightPosition);
}

// Function to debug motor signals - call from main loop if needed
void debugMotorSignals() {
  Serial.print("Left Dir Pin: ");
  Serial.print(digitalRead(leftDirPin));
  Serial.print(" | Left Throttle: ");
  Serial.print(analogRead(leftThrottlePin));
  Serial.print(" | Right Dir Pin: ");
  Serial.print(digitalRead(rightDirPin));
  Serial.print(" | Right Throttle: ");
  Serial.println(analogRead(rightThrottlePin));
}

// Test functions for individual motors
void testLeftMotorForward() {
  // Stop all motors first
  stop();
  
  // Set left motor to forward direction
  digitalWrite(leftDirPin, LOW);  // Forward
  delay(1000);
  analogWrite(leftThrottlePin, currentSpeed);
  analogWrite(rightThrottlePin, 0);  // Right motor off
  
  Serial.println("Left motor running forward");
  Serial.print("Direction pin state: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Speed: ");
  Serial.println(currentSpeed);
  
  // Run for 3 seconds then stop
  delay(3000);
  stop();
  Serial.println("Test complete");
}

void testLeftMotorBackward() {
  // Stop all motors first
  stop();
  
  // Set left motor to backward direction
  digitalWrite(leftDirPin, HIGH);  // Backward
  delay(1000);
  analogWrite(leftThrottlePin, currentSpeed);
  analogWrite(rightThrottlePin, 0);  // Right motor off
  
  Serial.println("Left motor running backward");
  Serial.print("Direction pin state: ");
  Serial.println(digitalRead(leftDirPin));
  Serial.print("Speed: ");
  Serial.println(currentSpeed);
  
  // Run for 3 seconds then stop
  delay(3000);
  stop();
  Serial.println("Test complete");
}

void testRightMotorForward() {
  // Stop all motors first
  stop();
  
  // Set right motor to forward direction
  digitalWrite(rightDirPin, LOW);  // Forward
  delay(1000);
  analogWrite(rightThrottlePin, currentSpeed);
  analogWrite(leftThrottlePin, 0);  // Left motor off
  
  Serial.println("Right motor running forward");
  Serial.print("Direction pin state: ");
  Serial.println(digitalRead(rightDirPin));
  Serial.print("Speed: ");
  Serial.println(currentSpeed);
  
  // Run for 3 seconds then stop
  delay(3000);
  stop();
  Serial.println("Test complete");
}

void testRightMotorBackward() {
  // Stop all motors first
  stop();
  
  // Set right motor to backward direction
  digitalWrite(rightDirPin, HIGH);  // Backward
  delay(1000);
  analogWrite(rightThrottlePin, currentSpeed);
  analogWrite(leftThrottlePin, 0);  // Left motor off
  
  Serial.println("Right motor running backward");
  Serial.print("Direction pin state: ");
  Serial.println(digitalRead(rightDirPin));
  Serial.print("Speed: ");
  Serial.println(currentSpeed);
  
  // Run for 3 seconds then stop
  delay(3000);
  stop();
  Serial.println("Test complete");
}

// Special test functions
void toggleLeftDirPin() {
  // Get current pin state
  int currentState = digitalRead(leftDirPin);
  
  // Set to opposite state
  digitalWrite(leftDirPin, !currentState);
  
  Serial.print("LEFT direction pin toggled from ");
  Serial.print(currentState);
  Serial.print(" to ");
  Serial.println(digitalRead(leftDirPin));
  
  // Show the pin state
  Serial.println("Current status:");
  debugMotorSignals();
}

void toggleRightDirPin() {
  // Get current pin state  
  int currentState = digitalRead(rightDirPin);
  
  // Set to opposite state
  digitalWrite(rightDirPin, !currentState);
  
  Serial.print("RIGHT direction pin toggled from ");
  Serial.print(currentState);
  Serial.print(" to ");
  Serial.println(digitalRead(rightDirPin));
  
  // Show the pin state
  Serial.println("Current status:");
  debugMotorSignals();
}

void specialBackwardTest() {
  // This is a test sequence to help debug backward movement
  Serial.println("STARTING SPECIAL BACKWARD TEST SEQUENCE");
  
  // Step 1: Stop everything
  stop();
  Serial.println("1. All motors stopped");
  debugMotorSignals();
  delay(1000);
  
  // Step 2: Set direction pins HIGH (backward)
  digitalWrite(leftDirPin, HIGH);
  digitalWrite(rightDirPin, HIGH);
  Serial.println("2. Direction pins set HIGH (backward)");
  debugMotorSignals();
  delay(1000);
  
  // Step 3: Verify direction pins
  Serial.println("3. Verifying direction pins");
  Serial.print("Left Dir Pin: ");
  Serial.println(digitalRead(leftDirPin) == HIGH ? "HIGH (correct)" : "LOW (wrong!)");
  Serial.print("Right Dir Pin: ");
  Serial.println(digitalRead(rightDirPin) == HIGH ? "HIGH (correct)" : "LOW (wrong!)");
  delay(1000);
  
  // Step 4: Apply throttle gradually
  Serial.println("4. Applying throttle gradually");
  for (int i = 50; i <= currentSpeed; i += 50) {
    analogWrite(leftThrottlePin, i);
    analogWrite(rightThrottlePin, i);
    Serial.print("Speed: ");
    Serial.println(i);
    delay(500);
  }
  
  // Step 5: Run at full configured speed
  Serial.println("5. Running at set speed");
  debugMotorSignals();
  delay(3000);
  
  // Step 6: Stop
  stop();
  Serial.println("6. Test complete");
}
