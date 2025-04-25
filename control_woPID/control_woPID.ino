#include <AccelStepper.h>
 
// Pin definitions
#define STEP_PIN 23 //right
#define DIR_PIN 25
#define ENCODER_A 22
#define ENCODER_B 26
#define STEP_PIN2 53 //left
#define DIR_PIN2 51
#define ENCODER_C 46
#define ENCODER_D 50
#define PWM_INPUT 37       // Throttle control
#define TURN_PIN 39        // Steering control
#define EMERGENCY_PIN 41   // Emergency shutdown input
#define ENA_RIGHT 27       // Right motor enable
#define ENA_LEFT 49        // Left motor enable
 
// Alert system pins
#define TRIG_RED 10
#define ECHO_RED 9
#define RED_LED 8
#define BUZZER 11
#define ALERT_DISTANCE 4.0
 
bool redAlert = false;
float currentDistance = 0.0;
unsigned long lastDistanceCheck = 0;
bool emergencyStopActive = false;
 
// Stepper motor setups
AccelStepper stepper(1, STEP_PIN, DIR_PIN);   // Left motor
AccelStepper stepper2(1, STEP_PIN2, DIR_PIN2); // Right motor
 
// Encoder variables
volatile long encoderCount = 0;
volatile long encoderCount2 = 0;
int lastAState = LOW;
int lastCState = LOW;
 
// PWM control variables
volatile unsigned long throttleStart = 0;
volatile unsigned long throttleValue = 1500;
volatile unsigned long steerStart = 0;
volatile unsigned long steerValue = 1500;
volatile unsigned long emergencyStart = 0;
volatile unsigned long emergencyValue = 1500;
bool needsArming = false;
 
// State management variables
char activeCommand = 'N'; // 'N' none, 'T' throttle, 'S' steering
unsigned long throttleActivationTime = 0;
unsigned long steerActivationTime = 0;
 
// System constants
const float STEPS_PER_REV = 121600.0;
const float ENCODER_TICKS_PER_REV = 38000.0;
const float WHEEL_CIRCUMFERENCE = 0.41;
const float MAX_SPEED = 50000.0;
const int DEAD_ZONE = 50;
const int EMERGENCY_DEAD_ZONE = 10; // 10μs dead zone for emergency stop
const int MOTOR1_DIRECTION = -1;  // Left motor direction
const int MOTOR2_DIRECTION = -1;  // Right motor direction
 
// Timing variables
unsigned long lastCheckTime = 0;
long lastEncoderCount = 0;
long lastEncoderCount2 = 0;
 
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0133) / 2;
}
 
class InterruptWrapper {
public:
  static void readThrottle() {
    if (digitalRead(PWM_INPUT)) {
      throttleStart = micros();
    } else {
      if (throttleStart != 0) {
        throttleValue = micros() - throttleStart;
        throttleStart = 0;
      }
    }
  }
 
  static void readSteering() {
    if (digitalRead(TURN_PIN)) {
      steerStart = micros();
    } else {
      if (steerStart != 0) {
        steerValue = micros() - steerStart;
        steerStart = 0;
      }
    }
  }
 
  static void readEmergency() {
    if (digitalRead(EMERGENCY_PIN)) {
      emergencyStart = micros();
    } else {
      if (emergencyStart != 0) {
        emergencyValue = micros() - emergencyStart;
        emergencyStart = 0;
      }
    }
  }
};
 
void setup() {
  Serial.begin(115200);
 
  pinMode(TRIG_RED, OUTPUT);
  pinMode(ECHO_RED, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  // Motor enable pins
  pinMode(ENA_LEFT, OUTPUT);
  pinMode(ENA_RIGHT, OUTPUT);
  digitalWrite(ENA_LEFT, HIGH);  // Initially disabled (active low)
  digitalWrite(ENA_RIGHT, HIGH); // Initially disabled (active low)
 
  // Encoder setup
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);
 
  // PWM inputs
  pinMode(PWM_INPUT, INPUT);
  pinMode(TURN_PIN, INPUT);
  pinMode(EMERGENCY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT), InterruptWrapper::readThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TURN_PIN), InterruptWrapper::readSteering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), InterruptWrapper::readEmergency, CHANGE);
 
  // Stepper configurations
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(5000);
  stepper.disableOutputs();
  
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(5000);
  stepper2.disableOutputs();
 
  Serial.println("Dual Motor System Ready - Exclusive Control");
  Serial.println("Send 'n' for emergency stop");
}
 
void loop() {
  checkEmergencyStop();
  
  if (millis() - lastDistanceCheck >= 100) {
    currentDistance = getDistance(TRIG_RED, ECHO_RED);
    redAlert = currentDistance <= ALERT_DISTANCE;
    digitalWrite(RED_LED, redAlert ? HIGH : LOW);
    tone(BUZZER, redAlert ? 3500 : 0);
    lastDistanceCheck = millis();
  }
 
  if (!emergencyStopActive) {
    checkSerial();
    updateEncoders();
    controlMotors();
    logVelocities();
  }
}
 
void checkEmergencyStop() {
  unsigned long emergency = constrain(emergencyValue, 1000, 2000);
  
  // Add 10μs dead zone at both ends
  bool emergencyCondition = (emergency <= (1000 + EMERGENCY_DEAD_ZONE)) ||
                          (emergency >= (2000 - EMERGENCY_DEAD_ZONE));
  
  if (emergencyCondition) {
    if (!emergencyStopActive) {
      emergencyStopActive = true;
      digitalWrite(ENA_LEFT, HIGH);   // Disable left motor
      digitalWrite(ENA_RIGHT, HIGH);  // Disable right motor
      stepper.disableOutputs();
      stepper2.disableOutputs();
      Serial.println("EMERGENCY STOP ACTIVATED VIA PWM SIGNAL");
    }
  } else {
    if (emergencyStopActive) {
      emergencyStopActive = false;
      needsArming = true;
      Serial.println("Emergency condition cleared, needs re-arming");
    }
  }
}
 
void updateEncoders() {
  int currentA = digitalRead(ENCODER_A);
  if (currentA != lastAState) {
    encoderCount += (digitalRead(ENCODER_B) != currentA) ? 1 : -1;
    lastAState = currentA;
  }
 
  int currentC = digitalRead(ENCODER_C);
  if (currentC != lastCState) {
    encoderCount2 += (digitalRead(ENCODER_D) != currentC) ? 1 : -1;
    lastCState = currentC;
  }
}
 
void controlMotors() {
  if (needsArming) {
    handleArming();
    return;
  }
 
  unsigned long throttle = constrain(throttleValue, 1000, 2000);
  unsigned long steering = constrain(steerValue, 1000, 2000);
  int throttleDiff = throttle - 1500;
  int steerDiff = steering - 1500;
 
  bool throttleActive = abs(throttleDiff) > DEAD_ZONE;
  bool steerActive = abs(steerDiff) > DEAD_ZONE;
 
  if (throttleActive) {
    if (throttleActivationTime == 0) throttleActivationTime = millis();
  } else {
    throttleActivationTime = 0;
  }
 
  if (steerActive) {
    if (steerActivationTime == 0) steerActivationTime = millis();
  } else {
    steerActivationTime = 0;
  }
 
  if (activeCommand == 'N') {
    if (throttleActive || steerActive) {
      if (throttleActive && !steerActive) activeCommand = 'T';
      else if (steerActive && !throttleActive) activeCommand = 'S';
      else {
        if (throttleActivationTime < steerActivationTime) activeCommand = 'T';
        else if (steerActivationTime < throttleActivationTime) activeCommand = 'S';
        else activeCommand = abs(throttleDiff) >= abs(steerDiff) ? 'T' : 'S';
      }
    }
  } else {
    if ((activeCommand == 'T' && !throttleActive) || (activeCommand == 'S' && !steerActive)) {
      activeCommand = 'N';
    }
  }
 
  switch (activeCommand) {
    case 'T': handleThrottle(throttleDiff); break;
    case 'S': handleSteering(steerDiff); break;
    default: stopMotors(); break;
  }
}
 
void handleThrottle(int throttleDiff) {
  if (redAlert) {
    // Reverse direction when alert is active
    throttleDiff = constrain(throttleDiff, 0, 500);
    float forwardSpeed = abs(throttleDiff/500.0) * MAX_SPEED;
    
    // Modified direction for forward movement during alert
    stepper.setSpeed(forwardSpeed * MOTOR1_DIRECTION);
    stepper2.setSpeed(-forwardSpeed * MOTOR2_DIRECTION);
  } else {
    float speed = (throttleDiff / 500.0) * MAX_SPEED;
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    stepper.setSpeed(speed * MOTOR1_DIRECTION);
    stepper2.setSpeed(-speed * MOTOR2_DIRECTION);
  }
  enableMotors();
}
 
void handleSteering(int steerDiff) {
  if (!redAlert) {
    float turnSpeed = (steerDiff / 500.0) * MAX_SPEED;
    turnSpeed = constrain(turnSpeed, -MAX_SPEED, MAX_SPEED);
 
    if(steerDiff > 0) {
      stepper.setSpeed(turnSpeed * MOTOR1_DIRECTION);
      stepper2.setSpeed(turnSpeed * MOTOR2_DIRECTION);
    } else {
      stepper.setSpeed(turnSpeed * MOTOR1_DIRECTION);
      stepper2.setSpeed(turnSpeed * MOTOR2_DIRECTION);
    }
    enableMotors();
  }
}
 
void enableMotors() {
  digitalWrite(ENA_LEFT, LOW);    // Enable left motor
  digitalWrite(ENA_RIGHT, LOW);   // Enable right motor
  if (!stepper.isRunning()) stepper.enableOutputs();
  if (!stepper2.isRunning()) stepper2.enableOutputs();
  stepper.runSpeed();
  stepper2.runSpeed();
}
 
void stopMotors() {
  stepper.stop();
  stepper2.stop();
  stepper.disableOutputs();
  stepper2.disableOutputs();
  digitalWrite(ENA_LEFT, HIGH);   // Disable left motor
  digitalWrite(ENA_RIGHT, HIGH);  // Disable right motor
}
 
void handleArming() {
  if (throttleValue >= 1490 && throttleValue <= 1510 && !emergencyStopActive) {
    needsArming = false;
    encoderCount = encoderCount2 = 0;
    stepper.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    activeCommand = 'N';
    throttleActivationTime = 0;
    steerActivationTime = 0;
    Serial.println("System re-armed");
  } else {
    stopMotors();
  }
}
 
void checkSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'n') {
      emergencyStop();
    }
    while (Serial.available()) Serial.read();
  }
}
 
void emergencyStop() {
  needsArming = true;
  emergencyStopActive = true;
  stopMotors();
  encoderCount = encoderCount2 = 0;
  stepper.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  activeCommand = 'N';
  throttleActivationTime = 0;
  steerActivationTime = 0;
  Serial.println("EMERGENCY STOP - Return to neutral to re-arm");
}
 
void logVelocities() {
  if (millis() - lastCheckTime >= 200) {
    float deltaTime = (millis() - lastCheckTime) / 1000.0;
    if(deltaTime <= 0) deltaTime = 0.0001;
 
    unsigned long currentThrottle, currentSteering, currentEmergency;
    noInterrupts();
    currentThrottle = throttleValue;
    currentSteering = steerValue;
    currentEmergency = emergencyValue;
    interrupts();
    
    currentThrottle = constrain(currentThrottle, 1000, 2000);
    currentSteering = constrain(currentSteering, 1000, 2000);
    currentEmergency = constrain(currentEmergency, 1000, 2000);
 
    float deltaTicks = encoderCount - lastEncoderCount;
    float velocity = ((deltaTicks / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE) / deltaTime;
    
    float deltaTicks2 = encoderCount2 - lastEncoderCount2;
    float velocity2 = ((deltaTicks2 / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE) / deltaTime;
 
    if(isnan(velocity)) velocity = 0;
    if(isnan(velocity2)) velocity2 = 0;
 
    Serial.print("Active: ");
    Serial.print(activeCommand);
    Serial.print(" | Throttle: ");
    Serial.print(currentThrottle);
    Serial.print("μs | Steering: ");
    Serial.print(currentSteering);
    Serial.print("μs | Emergency: ");
    Serial.print(currentEmergency);
    Serial.print("μs | M1: ");
    Serial.print(encoderCount);
    Serial.print("ticks (");
    Serial.print(velocity * MOTOR1_DIRECTION, 4);
    Serial.print("m/s) | M2: ");
    Serial.print(encoderCount2);
    Serial.print("ticks (");
    Serial.print(velocity2 * MOTOR2_DIRECTION, 4);
    Serial.print("m/s) | Dist: ");
    Serial.print(currentDistance);
    Serial.print("in | EStop: ");
    Serial.println(emergencyStopActive ? "YES" : "NO");
 
    lastEncoderCount = encoderCount;
    lastEncoderCount2 = encoderCount2;
    lastCheckTime = millis();
  }
}