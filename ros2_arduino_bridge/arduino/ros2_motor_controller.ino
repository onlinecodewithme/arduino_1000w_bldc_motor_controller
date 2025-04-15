/* 
 * ROS2 Compatible Arduino Motor Controller
 * 
 * This sketch provides a bridge between ROS2 and Arduino-controlled motors
 * for a tracked robot. It handles velocity commands, reads hall effect sensors
 * for odometry, and provides a serial interface for manual control.
 */

// Pin definitions
const int leftThrottlePin = 9;   // PWM output to left throttle
const int rightThrottlePin = 10; // PWM output to right throttle
const int leftDirPin = 7;        // Left direction control
const int rightDirPin = 8;       // Right direction control
const int leftHallPins[] = {2, 3, 4};    // Left motor hall sensors
const int rightHallPins[] = {5, 6, 12};  // Right motor hall sensors

// Position tracking
volatile long leftPosition = 0;
volatile long rightPosition = 0;
// Distance tracking (in encoder ticks converted to meters)
float leftDistance = 0.0;
float rightDistance = 0.0;
// Wheel parameters for distance calculation
const float ticksPerRevolution = 360.0;  // Adjust to your encoder resolution
const float wheelDiameter = 0.165;       // Wheel diameter in meters (adjust to your wheel size)
const float metersPerTick = (PI * wheelDiameter) / ticksPerRevolution;

// Movement parameters
int baseSpeed = 150;      // Base movement speed (0-255)
int turnDifference = 50;  // Speed difference for turning
bool moving = false;      // Movement state
int currentSpeed = 150;   // Current speed value (0-255)

// Serial communication
const long baudRate = 115200;
unsigned long lastOdomTime = 0;
const unsigned long odomPeriod = 100;  // Send odometry every 100ms
char buffer[64];
int bufferIndex = 0;

// ROS2 protocol constants
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
  stopMotors();
  
  // Start serial communication
  Serial.begin(baudRate);
  while (!Serial && millis() < 5000) {
    // Wait for serial port to connect (up to 5 seconds)
  }
  
  // Send initialization message
  Serial.println("ROS2 Motor Controller Initialized");
  printMenu();
}

void loop() {
  // Check for serial commands
  readSerial();

  // Periodically send odometry data
  unsigned long currentTime = millis();
  if (currentTime - lastOdomTime >= odomPeriod) {
    sendOdometry();
    lastOdomTime = currentTime;
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Check for special protocol headers
    if (c == COMMAND_HEADER) {
      // Next character is a manual command
      while (!Serial.available()) { /* Wait for command */ }
      processCommand(Serial.read());
      // Send acknowledgement
      Serial.print(ACK_HEADER);
      Serial.println(" Command processed");
    } 
    else if (c == VELOCITY_HEADER) {
      // Velocity command format: V[4-bytes float left_vel][4-bytes float right_vel]
      while (Serial.available() < 8) { /* Wait for complete data */ }
      
      // Read binary float data (4 bytes each for left and right velocity)
      byte leftBytes[4];
      byte rightBytes[4];
      
      Serial.readBytes(leftBytes, 4);
      Serial.readBytes(rightBytes, 4);
      
      // Convert to floats (assuming little-endian)
      float leftVel, rightVel;
      memcpy(&leftVel, leftBytes, 4);
      memcpy(&rightVel, rightBytes, 4);
      
      // Process velocity command
      setMotorVelocities(leftVel, rightVel);
    }
    else if (c == RESET_HEADER) {
      // Reset odometry
      resetOdometry();
      // Send acknowledgement
      Serial.print(ACK_HEADER);
      Serial.println(" Odometry reset");
    }
    else if (c == DEBUG_HEADER) {
      // Send debug information
      sendDebugInfo();
    }
    else if (c >= 32) {  // Only process visible characters (ignore line feeds, etc.)
      processCommand(c);
    }
  }
}

void processCommand(char cmd) {
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
      stopMotors();
      break;
    case '+': // Increase speed
      currentSpeed = min(255, currentSpeed + 10);
      Serial.print("Speed increased to: ");
      Serial.println(currentSpeed);
      updateMovement();
      break;
    case '-': // Decrease speed
      currentSpeed = max(0, currentSpeed - 10);
      Serial.print("Speed decreased to: ");
      Serial.println(currentSpeed);
      updateMovement();
      break;
    case 'c': // Synchronize tracks
      Serial.println("Synchronizing tracks");
      synchronizeTracks();
      break;
    case '?': // Print menu
      printMenu();
      break;
    default:
      // Ignore other characters
      if (cmd >= 32) { // Only print for visible characters
        Serial.print("Unknown command: ");
        Serial.println(cmd);
      }
      break;
  }
}

void printMenu() {
  Serial.println("\n=== ROS2 Motor Controller ===");
  Serial.println("Manual Controls:");
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
  Serial.print("Current speed: ");
  Serial.println(currentSpeed);
}

void updateMovement() {
  // Re-apply the current movement with the new speed
  if (moving) {
    // Detect current state based on pin values
    bool leftDir = digitalRead(leftDirPin);
    bool rightDir = digitalRead(rightDirPin);
    
    if (leftDir == LOW && rightDir == LOW) {
      moveForward(currentSpeed);
    } else if (leftDir == HIGH && rightDir == HIGH) {
      moveBackward(currentSpeed);
    } else if (leftDir == HIGH && rightDir == LOW) {
      turnLeft(currentSpeed);
    } else if (leftDir == LOW && rightDir == HIGH) {
      turnRight(currentSpeed);
    }
  }
}

// Basic movement functions
void moveForward(int speed) {
  digitalWrite(leftDirPin, LOW);    // Forward direction
  digitalWrite(rightDirPin, LOW);   // Forward direction
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void moveBackward(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Backward direction
  digitalWrite(rightDirPin, HIGH);  // Backward direction
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void turnLeft(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Left track backward
  digitalWrite(rightDirPin, LOW);   // Right track forward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void turnRight(int speed) {
  digitalWrite(leftDirPin, LOW);    // Left track forward
  digitalWrite(rightDirPin, HIGH);  // Right track backward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void pivotLeft(int speed) {
  digitalWrite(leftDirPin, HIGH);   // Left track backward
  digitalWrite(rightDirPin, LOW);   // Right track forward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void pivotRight(int speed) {
  digitalWrite(leftDirPin, LOW);    // Left track forward
  digitalWrite(rightDirPin, HIGH);  // Right track backward
  analogWrite(leftThrottlePin, speed);
  analogWrite(rightThrottlePin, speed);
  moving = true;
}

void stopMotors() {
  analogWrite(leftThrottlePin, 0);
  analogWrite(rightThrottlePin, 0);
  // Keep direction pins in their current state
  moving = false;
}

// ROS2 interface functions
void setMotorVelocities(float leftVel, float rightVel) {
  // Convert m/s velocities to motor speeds and directions
  int leftSpeed = constrain(abs(leftVel) * 255.0, 0, 255);
  int rightSpeed = constrain(abs(rightVel) * 255.0, 0, 255);
  
  // Set directions based on velocity signs
  digitalWrite(leftDirPin, leftVel < 0 ? HIGH : LOW);
  digitalWrite(rightDirPin, rightVel < 0 ? HIGH : LOW);
  
  // Set speeds
  analogWrite(leftThrottlePin, leftSpeed);
  analogWrite(rightThrottlePin, rightSpeed);
  
  moving = (leftSpeed > 0 || rightSpeed > 0);
  
  // Update current speed for manual control
  currentSpeed = max(leftSpeed, rightSpeed);
}

void sendOdometry() {
  // Calculate distance in meters
  leftDistance = leftPosition * metersPerTick;
  rightDistance = rightPosition * metersPerTick;
  
  // Format: O,left_pos,right_pos,left_dist,right_dist
  Serial.print(ODOMETRY_HEADER);
  Serial.print(",");
  Serial.print(leftPosition);
  Serial.print(",");
  Serial.print(rightPosition);
  Serial.print(",");
  Serial.print(leftDistance, 6);  // 6 decimal places
  Serial.print(",");
  Serial.println(rightDistance, 6);
}

void resetOdometry() {
  noInterrupts();  // Disable interrupts during reset
  leftPosition = 0;
  rightPosition = 0;
  leftDistance = 0.0;
  rightDistance = 0.0;
  interrupts();    // Re-enable interrupts
}

void sendDebugInfo() {
  // Send debug information about current pin states
  Serial.print(DEBUG_HEADER);
  Serial.print(",Moving:");
  Serial.print(moving);
  Serial.print(",Speed:");
  Serial.print(currentSpeed);
  Serial.print(",LeftDir:");
  Serial.print(digitalRead(leftDirPin));
  Serial.print(",RightDir:");
  Serial.print(digitalRead(rightDirPin));
  Serial.print(",LeftThrottle:");
  Serial.print(analogRead(leftThrottlePin));
  Serial.print(",RightThrottle:");
  Serial.print(analogRead(rightThrottlePin));
  Serial.print(",LeftPos:");
  Serial.print(leftPosition);
  Serial.print(",RightPos:");
  Serial.println(rightPosition);
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
      while (leftPosition - rightPosition > 5 && Serial.available() == 0) {
        delay(10);
      }
    } else {
      // Right track ahead, move left track to catch up
      digitalWrite(leftDirPin, LOW);    // Move left track forward
      digitalWrite(rightDirPin, LOW);   // Hold right track
      analogWrite(leftThrottlePin, 100);
      analogWrite(rightThrottlePin, 0);
      
      // Wait until synchronized (or abort if new command received)
      while (rightPosition - leftPosition > 5 && Serial.available() == 0) {
        delay(10);
      }
    }
    
    // Stop both tracks after synchronization
    stopMotors();
    Serial.println("Tracks synchronized!");
  } else {
    Serial.println("Tracks already in sync.");
  }
}

// Interrupt handlers for position tracking
void leftHallChange() {
  // Update position based on direction
  // IMPORTANT: This logic needs to match your motor controller's behavior
  // If using HIGH for backward, LOW for forward:
  if (digitalRead(leftDirPin) == LOW) {
    leftPosition++;
  } else {
    leftPosition--;
  }
}

void rightHallChange() {
  // Update position based on direction
  // IMPORTANT: This logic needs to match your motor controller's behavior
  // If using HIGH for backward, LOW for forward:
  if (digitalRead(rightDirPin) == LOW) {
    rightPosition++;
  } else {
    rightPosition--;
  }
}
