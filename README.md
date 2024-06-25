# Project Overview

This project involves developing a self-balancing robot that maintains its vertical position on two wheels using a gyroscope, accelerometer, and stepper motors controlled by a microcontroller. The goal is to implement control algorithms (PID) and explore applications in navigation, drone stabilization, and satellite orientation.

<div style="display: flex; justify-content: space-between;">
  <img src="https://github.com/OctavianMihaila/Self-balancing-robot/blob/master/p1_sbr.jpeg?raw=true" alt="Photo 1" width="200"/>
  <img src="https://github.com/OctavianMihaila/Self-balancing-robot/blob/master/p2_sbr.jpeg?raw=true" alt="Photo 2" width="200"/>
  <img src="https://github.com/OctavianMihaila/Self-balancing-robot/blob/master/p3_sbr.jpeg?raw=true" alt="Photo 3" width="200"/>
  <img src="https://github.com/OctavianMihaila/Self-balancing-robot/blob/master/p4_sbr.jpeg?raw=true" alt="Photo 3" width="200"/>
  
</div>

### Video

![Click to play video](https://www.youtube.com/watch?v=EH0yqp-uJCQ)


## Hardware Design

- Two stepper motors are controlled by A4988 drivers using microstepping.
- An MPU6050 gyroscope-accelerometer module provides tilt and orientation data.
- Motors and the Arduino board are powered separately by a 3S 11.1V battery, facilitating debugging and emergency power cutoff via a relay.

## Software Design

### MPU-6050 Interaction

- Uses I2C communication.
- Accelerometer sensitivity: ±8g.
- Gyroscope sensitivity: ±500 dps.
- Composite angle calculation involves gyroscope and accelerometer calibration, drift correction, and complementary filtering.

### PID Controller Implementation

The PID algorithm processes the composite angle:

- pid_result = PID_KP * curr_pid_error + PID_KI * (curr_pid_error + prev_pid_error) + PID_KD * (curr_pid_error - prev_pid_error)

- Output is limited to ±MAX_PID_RESULT.
- PID constants tuning:
  - PID_KP for oscillation control.
  - PID_KI for correcting accumulated error.
  - PID_KD for stability at the balance point and vibration elimination.

### Stepper Motor Control

A custom library, using direct register manipulation, generates pulses based on the PID algorithm results.

### Battery Management

- Voltage is reduced below 5V via a voltage divider (R1 = 10KΩ, R2 = 4.7KΩ) to be read on an analog pin.
- Battery voltage is monitored, and a red LED lights up when it drops below 11.5V.
