Dual Motor Control System

Overview
This Arduino project controls a dual-motor system using stepper motors, with PWM inputs for throttle and steering, encoder feedback for velocity tracking, 
and an ultrasonic sensor for obstacle detection. It includes an emergency stop mechanism and an alert system with a buzzer and LED. The project provides two
control versions: Open Loop and Closed Loop (PID).

Features
Dual Stepper Motor Control: Drives two stepper motors for differential drive (left and right wheels).
Control Modes:
  Open Loop: Direct speed control based on PWM inputs.
  Closed Loop (PID): Uses encoder feedback for precise velocity control with PID tuning.
PWM Inputs: Throttle (speed), steering (direction), and emergency stop control via PWM signals (1000–2000 μs).
Encoder Feedback: Tracks motor position and calculates wheel velocity.
Obstacle Detection: Ultrasonic sensor triggers an alert (LED and buzzer) when objects are within 4 inches.
Emergency Stop: Disables motors via PWM signal or serial command ('n').
Serial Monitoring: Logs motor velocities, PWM values, and system state.

Hardware Requirements
Microcontroller: Arduino-compatible board (e.g., Arduino Mega R1).
Motors: Two stepper motors with drivers (e.g., A4988 or DRV8825).
Encoders: Quadrature encoders for each motor.
Sensors: HC-SR04 ultrasonic sensor.
Other Components: LED, buzzer, PWM signal source (e.g., RC receiver), and motor enable circuitry.

Pin Configuration: See pin definitions in the code (STEP_PIN, DIR_PIN, etc.).
Software Requirements
Arduino IDE: For compiling and uploading the code.
Libraries:
AccelStepper: For stepper motor control. Install via Arduino Library Manager.
(For Closed Loop) Additional PID library may be required (e.g., PID_v1).
Serial Monitor: Set baud rate to 115200 for debugging.

Installation
Arming: After an emergency stop, set throttle to neutral (1490–1510 μs) to re-arm the system.
Control:
  Throttle: PWM signal (1000–2000 μs) controls forward/backward speed.
  Steering: PWM signal (1000–2000 μs) controls turning.
  Emergency Stop: PWM signal outside 1000–2000 μs or serial command 'n' triggers stop.

Control Modes:
  Open Loop: Directly maps PWM inputs to motor speeds.
  Closed Loop (PID): Uses encoder feedback to maintain target velocities with PID control.
Obstacle Alert: Buzzer and LED activate if an object is within 10 inches.
Serial Output: Monitor motor velocities, PWM values, and system status via the Serial Monitor.
