# Self-Balancing Robot

A two-wheeled self-balancing robot built around an **ESP32**, designed to maintain its balance in real time while supporting wireless control and monitoring.

## Features

- Real-time self-balancing using an **MPU6050 IMU**
- PID-based balance control
- Wi-Fi remote control through a web dashboard
- Live monitoring of orientation, battery voltage, and Wi-Fi signal
- Adjustable PID parameters and balance angle
- Safety mechanisms for sensor and connection failures

## Project Structure

```text
.
├── index.html   # Web control and monitoring dashboard
├── main.cpp     # ESP32 firmware
└── README.md
```

## How It Works

The ESP32 reads motion data from the MPU6050 and continuously adjusts the motors to keep the robot upright. A browser-based interface allows the robot to be controlled and tuned wirelessly while displaying live telemetry.


## Hardware

**ESP32 · MPU6050 · L298N Motor Driver · TT DC Gear Motors · Battery**

## Acknowledgements

This project was developed collaboratively, combining the embedded control, hardware, and web interface components.
