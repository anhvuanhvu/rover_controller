#include <AccelStepper.h>
#include <PID_v1.h>

// Pin definitions
#define STEP_PIN 23        // Step pin for motor 1
#define DIR_PIN 25         // Direction pin for motor 1
#define ENCODER_A 22       // Encoder channel A for motor 1
#define ENCODER_B 26       // Encoder channel B for motor 1
#define STEP_PIN2 53       // Step pin for motor 2
#define DIR_PIN2 51        // Direction pin for motor 2
#define ENCODER_C 46       // Encoder channel A for motor 2
#define ENCODER_D 50       // Encoder channel B for motor 2
#define PWM_INPUT 37       // PWM input pin for target velocity

// Stepper motor setup
AccelStepper stepper(1, STEP_PIN, DIR_PIN);   // Motor 1: Driver mode
AccelStepper stepper2(1, STEP_PIN2, DIR_PIN2); // Motor 2: Driver mode

// Encoder variables
volatile long encoderCount = 0;   // Tracks encoder position for motor 1
volatile long encoderCount2 = 0;  // Tracks encoder position for motor 2
int lastAState = LOW;            // Last state of encoder A (motor 1)
int lastCState = LOW;            // Last state of encoder A (motor 2)

// PWM control variables
volatile unsigned long startPulse = 0; // Start time of PWM pulse
volatile unsigned long pwmValue = 1500; // PWM pulse width (default neutral)
bool needsArming = false;        // Arming flag for safety

// System constants
const float STEPS_PER_REV = 121600.0;      // Steps per motor revolution
const float ENCODER_TICKS_PER_REV = 38000.0; // Encoder ticks per revolution
const float WHEEL_CIRCUMFERENCE = 0.41;    // Wheel circumference in meters
const float MAX_SPEED = 75000.0;           // Max stepper speed (steps/sec)
const float MAX_ACTUAL_SPEED = 25000.0;    // Max actual speed (steps/sec)
const int DEAD_ZONE = 50;                  // PWM dead zone around neutral
const int MOTOR1_DIRECTION = -1;           // Motor 1 direction (-1 or 1)
const int MOTOR2_DIRECTION = 1;           // Motor 2 direction (-1 or 1)

// PID controller setup
double pidSetpoint, pidInput, pidOutput;   // PID variables for motor 1
double pidSetpoint2, pidInput2, pidOutput2; // PID variables for motor 2
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 
          60.0, 0.50, 0.0,                 // Kp, Ki, Kd for motor 1
          (MOTOR1_DIRECTION == -1) ? REVERSE : DIRECT); // PID direction for motor 1
PID myPID2(&pidInput2, &pidOutput2, &pidSetpoint2, 
           60.0, 0.0, 0.0,                // Kp, Ki, Kd for motor 2
           (MOTOR2_DIRECTION == -1) ? REVERSE : DIRECT); // PID direction for motor 2

// Velocity filtering and timing
const int VELOCITY_SAMPLES = 5;            // Number of samples for moving average
float velocityHistory[VELOCITY_SAMPLES];   // Velocity samples for motor 1
float velocityHistory2[VELOCITY_SAMPLES];  // Velocity samples for motor 2
int velocityIndex = 0;                     // Index for motor 1 velocity array
int velocityIndex2 = 0;                    // Index for motor 2 velocity array
unsigned long lastPIDTime = 0;             // Last PID update time
const unsigned long pidInterval = 25;      // PID update interval (ms)

// Velocity variables
float targetVelocity = 0;                  // Desired velocity (m/s)
float actualVelocity = 0;                  // Measured velocity for motor 1 (m/s)
float actualVelocity2 = 0;                 // Measured velocity for motor 2 (m/s)

// Debug mode flag
bool debugMode = false;

// Wrapper class for PWM interrupt
class InterruptWrapper {
public:
  static void calcPWM() {
    if (digitalRead(PWM_INPUT)) {
      startPulse = micros(); // Record pulse start time
    } else {
      if (startPulse != 0) {
        pwmValue = micros() - startPulse; // Calculate pulse width
        startPulse = 0;
      }
    }
  }
};

void setup() {
  Serial.begin(115200); // Start serial communication

  // Encoder setup
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), updateEncoder2, CHANGE);

  // PWM input setup
  pinMode(PWM_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT), InterruptWrapper::calcPWM, CHANGE);

  // Stepper motor configuration
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(10000);
  stepper.disableOutputs(); // Disable motor 1 at startup
  stepper2.setMaxSpeed(MAX_SPEED);
  stepper2.setAcceleration(10000);
  stepper2.disableOutputs(); // Disable motor 2 at startup

  // Initialize velocity filters
  for (int i = 0; i < VELOCITY_SAMPLES; i++) {
    velocityHistory[i] = 0;
    velocityHistory2[i] = 0;
  }

  // PID configuration
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  myPID.SetSampleTime(pidInterval);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
  myPID2.SetSampleTime(pidInterval);

  // Serial header for monitoring
  Serial.println("Target,Actual1,Actual2,PWM,Encoder1,Encoder2,PIDOutput1,PIDOutput2,Ref1,Ref-1");
}

void loop() {
  checkSerial(); // Check for serial commands
  if (debugMode) {
    // Debug mode: run both motors at fixed speed
    stepper.enableOutputs();
    stepper2.enableOutputs();
    stepper.setSpeed(50000);
    stepper2.setSpeed(50000);
    stepper.runSpeed();
    stepper2.runSpeed();
    logVelocities();
  } else {
    // Normal mode: run PID control
    updatePID();
    stepper.runSpeed();
    stepper2.runSpeed();
    logVelocities();
  }
}

// Encoder interrupt handler for motor 1
void updateEncoder() {
  int currentA = digitalRead(ENCODER_A);
  int currentB = digitalRead(ENCODER_B);
  if (currentA != lastAState) {
    int delta = (currentB != currentA) ? 1 : -1; // Determine direction
    static int lastDelta = 0;
    static int debounceCount = 0;
    if (delta == lastDelta) {
      debounceCount++;
      if (debounceCount >= 2) { // Simple debounce
        encoderCount += delta;
        debounceCount = 0;
      }
    } else {
      lastDelta = delta;
      debounceCount = 0;
    }
    lastAState = currentA;
  }
}

// Encoder interrupt handler for motor 2
void updateEncoder2() {
  int currentC = digitalRead(ENCODER_C);
  int currentD = digitalRead(ENCODER_D);
  if (currentC != lastCState) {
    int delta = (currentD != currentC) ? 1 : -1; // Determine direction
    static int lastDelta2 = 0;
    static int debounceCount2 = 0;
    if (delta == lastDelta2) {
      debounceCount2++;
      if (debounceCount2 >= 2) { // Simple debounce
        encoderCount2 += delta;
        debounceCount2 = 0;
      }
    } else {
      lastDelta2 = delta;
      debounceCount2 = 0;
    }
    lastCState = currentC;
  }
}

// Update PID controllers for both motors
void updatePID() {
  if (needsArming) {
    // Arming check: PWM must be neutral (1490-1510 µs)
    if (pwmValue >= 1490 && pwmValue <= 1510) {
      needsArming = false;
      encoderCount = 0;
      encoderCount2 = 0;
      stepper.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      myPID.SetMode(MANUAL);
      myPID2.SetMode(MANUAL);
      pidOutput = 0;
      pidOutput2 = 0;
      myPID.SetMode(AUTOMATIC);
      myPID2.SetMode(AUTOMATIC);
    }
    return;
  }

  if (millis() - lastPIDTime >= pidInterval) {
    lastPIDTime = millis();

    // Read PWM input safely
    unsigned long currentPWM;
    noInterrupts();
    currentPWM = pwmValue;
    interrupts();
    currentPWM = constrain(currentPWM, 1000, 2000); // Limit PWM range
    int pwmDifference = currentPWM - 1500;          // Offset from neutral

    // Calculate target velocity (m/s)
    targetVelocity = 0;
    if (abs(pwmDifference) >= DEAD_ZONE) {
      targetVelocity = (pwmDifference / 500.0) * (MAX_ACTUAL_SPEED * WHEEL_CIRCUMFERENCE / STEPS_PER_REV);
    }

    // Calculate actual velocity for motor 1 with moving average filter
    static long lastEncoderPID = 0;
    long deltaTicks = encoderCount - lastEncoderPID;
    lastEncoderPID = encoderCount;
    float dt = pidInterval / 1000.0; // Time interval in seconds

    float rawVelocity = (deltaTicks / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE / dt;
    velocityHistory[velocityIndex] = rawVelocity;
    velocityIndex = (velocityIndex + 1) % VELOCITY_SAMPLES;

    actualVelocity = 0;
    for (int i = 0; i < VELOCITY_SAMPLES; i++) {
      actualVelocity += velocityHistory[i];
    }
    actualVelocity /= VELOCITY_SAMPLES;

    // Zero small velocities to reduce noise
    if (abs(actualVelocity) < 0.03) actualVelocity = 0;

    // Calculate actual velocity for motor 2 with moving average filter
    static long lastEncoderPID2 = 0;
    long deltaTicks2 = encoderCount2 - lastEncoderPID2;
    lastEncoderPID2 = encoderCount2;

    float rawVelocity2 = -(deltaTicks2 / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE / dt;
    velocityHistory2[velocityIndex2] = rawVelocity2;
    velocityIndex2 = (velocityIndex2 + 1) % VELOCITY_SAMPLES;

    actualVelocity2 = 0;
    for (int i = 0; i < VELOCITY_SAMPLES; i++) {
      actualVelocity2 += velocityHistory2[i];
    }
    actualVelocity2 /= VELOCITY_SAMPLES;

    // Zero small velocities to reduce noise
    if (abs(actualVelocity2) < 0.03) actualVelocity2 = 0;

    // Convert velocities to steps/sec for PID
    pidSetpoint = targetVelocity * STEPS_PER_REV / WHEEL_CIRCUMFERENCE;
    pidInput = actualVelocity * STEPS_PER_REV / WHEEL_CIRCUMFERENCE;
    pidSetpoint2 = targetVelocity * STEPS_PER_REV / WHEEL_CIRCUMFERENCE;
    pidInput2 = actualVelocity2 * STEPS_PER_REV / WHEEL_CIRCUMFERENCE;

    // Compute PID outputs
    myPID.Compute();
    myPID2.Compute();

    // Apply output deadband and control motor 1
    float output = abs(pidOutput) < (0.3 * MAX_SPEED) ? 0 : pidOutput;
    if (output != 0) {
      stepper.enableOutputs();
      stepper.setSpeed(output);
    } else {
      stepper.setSpeed(0);
      stepper.stop();
      stepper.setCurrentPosition(encoderCount * (STEPS_PER_REV / ENCODER_TICKS_PER_REV));
      stepper.disableOutputs();
      if (targetVelocity == 0 && abs(pidInput) < 25) {
        pidOutput = 0;
        delay(5);
      }
    }

    // Apply output deadband and control motor 2
    float output2 = abs(pidOutput2) < (0.3 * MAX_SPEED) ? 0 : pidOutput2;
    if (output2 != 0) {
      stepper2.enableOutputs();
      stepper2.setSpeed(output2);
    } else {
      stepper2.setSpeed(0);
      stepper2.stop();
      stepper2.setCurrentPosition(encoderCount2 * (STEPS_PER_REV / ENCODER_TICKS_PER_REV));
      stepper2.disableOutputs();
      if (targetVelocity == 0 && abs(pidInput2) < 25) {
        pidOutput2 = 0;
        delay(5);
      }
    }
  }
}

// Log data to Serial Plotter
void logVelocities() {
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime >= 50) {
    unsigned long currentPWM;
    long currentEncoder, currentEncoder2;
    float currentOutput, currentOutput2;
    noInterrupts();
    currentPWM = pwmValue;
    currentEncoder = encoderCount;
    currentEncoder2 = encoderCount2;
    currentOutput = pidOutput;
    currentOutput2 = pidOutput2;
    interrupts();

    float displayActual = abs(actualVelocity) < 0.005 ? 0 : actualVelocity;
    float displayActual2 = abs(actualVelocity2) < 0.005 ? 0 : actualVelocity2;

    Serial.print(targetVelocity, 4);
    Serial.print(",");
    Serial.print(displayActual, 4);
    Serial.print(",");
    Serial.print(displayActual2, 4);
    Serial.print(",");
    Serial.print(currentPWM);
    Serial.print(",");
    Serial.print(currentEncoder);
    Serial.print(",");
    Serial.print(currentEncoder2);
    Serial.print(",");
    Serial.print(currentOutput, 4);
    Serial.print(",");
    Serial.print(currentOutput2, 4);
    Serial.print(",");
    Serial.print(0.1);  // Reference line at 0.1
    Serial.print(",");
    Serial.print(-0.1); // Reference line at -0.1
    Serial.println();

    lastLogTime = millis();
  }
}

// Handle serial commands
void checkSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'n') {
      emergencyStop(); // Emergency stop
    } else if (cmd == 'd') {
      debugMode = true; // Enter debug mode
      Serial.println("DEBUG MODE: Running at 50000 steps/s");
    } else if (cmd == 'p') {
      debugMode = false; // Return to PID mode
      stepper.disableOutputs();
      stepper2.disableOutputs();
      Serial.println("PID MODE");
    }
    while (Serial.available()) Serial.read(); // Clear buffer
  }
}

// Emergency stop function
void emergencyStop() {
  needsArming = true;
  stepper.stop();
  stepper2.stop();
  stepper.disableOutputs();
  stepper2.disableOutputs();
  myPID.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);
  pidOutput = 0;
  pidOutput2 = 0;
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  targetVelocity = 0;
  Serial.println("EMERGENCY STOP");
}