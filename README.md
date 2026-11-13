# Line Follower Robot v2.0

Second generation of my STM32 line-following robot: a redesigned sensor array and body PCB (Altium Designer), plus firmware that adds checkpoint-based path planning, wall following for tunnels, and sonar obstacle sensing. The first generation is in [Line-Follower](https://github.com/simulahmed003/Line-Follower), and the control analysis is in [System-Identification-and-PID-Control-Implementation-on-LFR](https://github.com/simulahmed003/System-Identification-and-PID-Control-Implementation-on-LFR).

## What is in this repo

| Folder | Contents |
|---|---|
| `LFR_Array_v2.0/` | Sensor-array board: schematic, PCB, output job and PDF, with a custom library for the 74HC4067 16-channel multiplexer module, TCRT5000 reflective sensors, sonar and headers |
| `LFR_Body_v2.0/` | Main body board: schematic and PCB |
| `Firmware/LFR_SH/` | Main firmware (Arduino-style C++ for STM32), split into modules |
| `Firmware/sonar/`, `Firmware/Adv_sonar_STM/`, `Firmware/MPU6050/` | Standalone test sketches for the sonar sensors and the MPU6050 IMU |
| `Guide/` | A beginner guide to line-follower robots and a soldering guide (PDF) |
| `image/` | Mechanical part (SolidWorks part and render) |

## Hardware

- A 16-channel TCRT5000 sensor array read through a 74HC4067 analog multiplexer on a single ADC input.
- Three ultrasonic sensors (left, middle, right) for walls and obstacles.
- Dual DC motor driver with PWM speed and direction pins. All pin assignments are in `Firmware/LFR_SH/config.h`.
- Buzzer, two status LEDs and two buttons for calibration and run control.

## Firmware structure (`Firmware/LFR_SH/`)

| File | Role |
|---|---|
| `LFR_SH.ino` | Entry point |
| `config.h` | Pin map for motors, mux, sonar, LEDs, buttons |
| `init.*` | Hardware and PID settings initialisation |
| `functions.*`, `globals.*` | Sensor reading, motor control helpers, shared state |
| `path_planing.*` | Line-following PID, checkpoint list and path sequence for the track |
| `tunnel.*` | Sonar reading (echo interrupt) and wall-following through tunnels |

## Build

Open `Firmware/LFR_SH/LFR_SH.ino` in the Arduino IDE with an STM32 core installed. Open the `.PrjPcb` files in Altium Designer; the libraries are included, so nothing else is needed.

## Notes

The `.rar` archives in `Firmware/` are earlier tuned versions and a CubeIDE project kept for reference.
