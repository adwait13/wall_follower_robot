# MIC PID Wall-Following Robot

This repository contains the code and documentation for a wall-following robot designed and built as part of the ME2400 course at IIT Madras. The robot uses an ultrasonic sensor for distance measurement and a PID controller to maintain a fixed distance from a wall as it navigates.

---

## Overview

The robot is a 3-wheeled differential drive system: two independently controlled DC motors and a passive caster wheel. It continuously monitors its distance from the wall and adjusts its wheel speeds using a PID feedback control loop to follow the wall at a set distance.

The robot also handles a turn sequence when it loses contact with the wall, enabling it to navigate corners or gaps.

---

## Features

- Real-time wall-following using PID control
- Distance sensing with HC-SR04 ultrasonic sensor
- Custom turn-and-forward behavior when wall contact is lost
- Manual tuning of controller gains inspired by the Ziegler–Nichols method
- Integral windup protection for long-range starts

---

## Hardware Used

- Arduino Uno
- L298N Motor Driver
- 2× Johnson DC motors with gearbox
- HC-SR04 Ultrasonic Distance Sensor
- Castor wheel
- Li-ion battery pack (11V)
- Custom wooden mount for caster wheel

---

## PID Tuning Approach

We used a manual, behavior-driven tuning method:

1. **Proportional gain (Kp)** was increased until the robot oscillated consistently around the wall.
2. **Derivative gain (Kd)** was added to dampen these oscillations and improve straight-line behavior.
3. **Integral gain (Ki)** was introduced to address minor steady-state errors, with a constraint to prevent windup when starting far from the wall.

> Final values used:  
> `Kp = 6`, `Ki = 0.009`, `Kd = 13.9`

---

## Contents

- Final Arduino code with PID logic  
- Problem Statement  
- Project Report (PDF)  
- `README.md`: This file

## License

This project is for academic and educational use. No formal license applied.

