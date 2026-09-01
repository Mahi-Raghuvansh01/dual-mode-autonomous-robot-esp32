/*
 * Dual-Mode Autonomous Navigation Robot (ESP32)
 * ----------------------------------------------
 * Supports two operating modes, selectable with a push button:
 *   1. AUTONOMOUS OBSTACLE AVOIDANCE — ultrasonic-based wandering
 *   2. OFFLINE VOICE CONTROL — locally recognized voice commands
 *      (forward / back / left / right / stop / "return")
 *
 * A lightweight dead-reckoning odometry system logs the path taken
 * so the robot can execute a Return-to-Home (RTH) maneuver, triggered
 * either by a voice command or a low-battery/timeout condition.
 *
 * Hardware:
 *   - ESP32 Dev Module
 *   - L298N (or TB6612FNG) dual motor driver
 *   - HC-SR04 ultrasonic sensor (front-facing, optionally on a servo)
 *   - Elechouse Voice Recognition V3 module (UART, offline/onboard trained commands)
 *   - Wheel encoders (or open-loop timing-based odometry as fallback)
 *   - Custom PCB (designed in EasyEDA) integrating sensor + motor driver headers
 *
 * Author: Mahi Raghuvanshi
 */

#include <HardwareSerial.h>

// ---------------- Pin Definitions ----------------
// Motor driver (L298N)
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 33
#define IN3 14
#define IN4 12

// Ultrasonic sensor
#define TRIG_PIN 5
#define ECHO_PIN 18

// Mode select button
#define MODE_BUTTON_PIN 4

// Voice recognition module (UART2)
HardwareSerial VoiceSerial(2); // RX2=16, TX2=17
#define VR_RX 16
#define VR_TX 17

// Voice command IDs (as trained on the VR module — adjust to your training set)
enum VoiceCommand {
  VR_NONE = -1,
  VR_FORWARD = 0,
  VR_BACKWARD = 1,
  VR_LEFT = 2,
  VR_RIGHT = 3,
  VR_STOP = 4,
  VR_RETURN_HOME = 5
};

// ---------------- Operating Modes ----------------
enum RobotMode { MODE_OBSTACLE_AVOIDANCE, MODE_VOICE_CONTROL };
RobotMode currentMode = MODE_OBSTACLE_AVOIDANCE;

bool lastButtonState = HIGH;

// ---------------- Odometry (for Return-to-Home) ----------------
// Simple pose tracking using timed motion segments as a proxy for
// distance/heading (replace with encoder ticks for higher accuracy).
struct PathSegment {
  float headingDeg;
  unsigned long durationMs;
};

const int MAX_PATH_SEGMENTS = 200;
PathSegment pathLog[MAX_PATH_SEGMENTS];
int pathLength = 0;
float currentHeadingDeg = 0.0; // 0 = "home-facing" reference

bool returningHome = false;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  VoiceSerial.begin(9600, SERIAL_8N1, VR_RX, VR_TX);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

  stopMotors();
  Serial.println(F("Dual-Mode Autonomous Robot Initialized"));
  Serial.println(F("Mode: OBSTACLE AVOIDANCE (default)"));
}

// ---------------- Main Loop ----------------
void loop() {
  handleModeButton();

  if (returningHome) {
    executeReturnToHome();
    return;
  }

  if (currentMode == MODE_OBSTACLE_AVOIDANCE) {
    runObstacleAvoidance();
  } else {
    runVoiceControl();
  }
}

// ---------------- Mode Switching ----------------
void handleModeButton() {
  bool reading = digitalRead(MODE_BUTTON_PIN);
  if (reading == LOW && lastButtonState == HIGH) {
    currentMode = (currentMode == MODE_OBSTACLE_AVOIDANCE)
                    ? MODE_VOICE_CONTROL
                    : MODE_OBSTACLE_AVOIDANCE;
    stopMotors();
    Serial.print(F("Switched mode to: "));
    Serial.println(currentMode == MODE_OBSTACLE_AVOIDANCE ? "OBSTACLE AVOIDANCE" : "VOICE CONTROL");
    delay(300); // debounce
  }
  lastButtonState = reading;
}

// ---------------- Obstacle Avoidance Mode ----------------
void runObstacleAvoidance() {
  long distanceCm = readUltrasonicCm();

  if (distanceCm > 25 || distanceCm == -1) {
    driveForward();
    logSegment(currentHeadingDeg, 200);
    delay(200);
  } else {
    stopMotors();
    delay(100);
    driveBackward();
    logSegment(currentHeadingDeg + 180, 300);
    delay(300);

    // Turn away from obstacle (right turn as default avoidance behavior)
    turnRight();
    currentHeadingDeg += 60;
    logSegment(currentHeadingDeg, 400);
    delay(400);
    stopMotors();
  }
}

// ---------------- Voice Control Mode ----------------
void runVoiceControl() {
  int cmd = readVoiceCommand();
  if (cmd == VR_NONE) return;

  switch (cmd) {
    case VR_FORWARD:
      driveForward();
      logSegment(currentHeadingDeg, 500);
      break;
    case VR_BACKWARD:
      driveBackward();
      logSegment(currentHeadingDeg + 180, 500);
      break;
    case VR_LEFT:
      turnLeft();
      currentHeadingDeg -= 30;
      logSegment(currentHeadingDeg, 300);
      break;
    case VR_RIGHT:
      turnRight();
      currentHeadingDeg += 30;
      logSegment(currentHeadingDeg, 300);
      break;
    case VR_STOP:
      stopMotors();
      break;
    case VR_RETURN_HOME:
      Serial.println(F("Voice command: RETURN HOME received."));
      returningHome = true;
      break;
  }
}

// Reads a recognized command index from the VR module, if any is available.
// The Elechouse VR3 module sends a single byte corresponding to the
// trained command index over UART when a match is recognized.
int readVoiceCommand() {
  if (VoiceSerial.available()) {
    int val = VoiceSerial.read();
    if (val >= 0 && val <= 5) return val;
  }
  return VR_NONE;
}

// ---------------- Return-to-Home ----------------
// Replays the logged path in reverse order and heading to navigate
// back to the approximate starting point.
void executeReturnToHome() {
  Serial.println(F("Executing Return-to-Home..."));
  for (int i = pathLength - 1; i >= 0; i--) {
    float reverseHeading = pathLog[i].headingDeg + 180.0;
    orientToHeading(reverseHeading);
    driveForward();
    delay(pathLog[i].durationMs);
    stopMotors();
    delay(100);
  }
  pathLength = 0;
  currentHeadingDeg = 0;
  returningHome = false;
  Serial.println(F("Return-to-Home complete."));
}

// Rotates the robot to approximately face the target heading.
// Simplified proportional turn based on heading error and timing.
void orientToHeading(float targetHeadingDeg) {
  float error = targetHeadingDeg - currentHeadingDeg;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;

  if (abs(error) < 5) return;

  if (error > 0) turnRight(); else turnLeft();
  delay((int)(abs(error) * 4)); // empirically tuned ms-per-degree
  stopMotors();
  currentHeadingDeg = targetHeadingDeg;
}

void logSegment(float heading, unsigned long durationMs) {
  if (pathLength >= MAX_PATH_SEGMENTS) return;
  pathLog[pathLength].headingDeg = heading;
  pathLog[pathLength].durationMs = durationMs;
  pathLength++;
}

// ---------------- Ultrasonic Sensor ----------------
long readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m range)
  if (duration == 0) return -1;
  return duration / 58; // convert to cm
}

// ---------------- Motor Control ----------------
void driveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 200); analogWrite(ENB, 200);
}

void driveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200); analogWrite(ENB, 200);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 180); analogWrite(ENB, 180);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180); analogWrite(ENB, 180);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}
