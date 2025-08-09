# Robotic Vehicle Embedded Systems (Fener Arduino)

<p align="center">
  <img src="https://user-images.githubusercontent.com/74321576/146676065-4ae206fa-0116-4e08-8090-edcc8398da87.jpg" alt="System Architecture" width="640" />
  <br/>
  <em>System architecture overview</em>
  <br/><br/>
  <a href="https://github.com/sezer-muhammed/Fener-Ard-Repo-v1">
    <img alt="Repo" src="https://img.shields.io/badge/repo-Fener--Ard--Repo--v1-24292e?style=flat-square&logo=github" />
  </a>
  <a href="https://www.arduino.cc/">
    <img alt="Platform" src="https://img.shields.io/badge/platform-Arduino%20Nano%20Every-00979D?style=flat-square&logo=arduino&logoColor=white" />
  </a>
  <a href="https://github.com/br3ttb/Arduino-PID-Library">
    <img alt="PID" src="https://img.shields.io/badge/lib-PID__V1-blue?style=flat-square" />
  </a>
  <a href="https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library">
    <img alt="PCA9685" src="https://img.shields.io/badge/servo-Adafruit%20PCA9685-ff69b4?style=flat-square&logo=adafruit&logoColor=white" />
  </a>
  <img alt="License" src="https://img.shields.io/badge/license-MIT-lightgrey?style=flat-square" />
</p>

A robust Arduino firmware for a robotic vehicle with dual-motor speed PID and Ackermann-style front steering driven by a PCA9685. It bridges RC input and a Jetson Nano (ROS 2) companion over high‑rate serial.

- MCU: Arduino Nano Every (ATmega4809)
- Companion: NVIDIA Jetson Nano (ROS 2 nodes)
- Key libs: PID_V1, Servo, Adafruit PWM Servo Driver, Wire

## Features

- Dual rear wheel closed-loop speed control (independent PID)
- Ackermann steering with two front servos via PCA9685
- RC teleop fallback (CH1 speed, CH2 angle) with smoothing
- Serial link to Jetson Nano: IMU in, encoder out, robust heading unwrap
- Tunable geometry and limits via `consts.h`

## Repository layout

- `fener.ino` — main loop, interrupts, PID setup, I/O wiring
- `functions.ino` — steering, RC processing, PID execution, protocol
- `consts.h` — geometric constants, pin map, limits

## Quick start

1) Clone

```bash
git clone https://github.com/sezer-muhammed/Fener-Ard-Repo-v1.git
```

2) Open in Arduino IDE
- Board: Arduino Nano Every
- Port: your device COMx

3) Libraries
- PID_V1 (br3ttb)
- Servo (builtin)
- Adafruit PWM Servo Driver (PCA9685)
- Wire (builtin)

4) Upload
- Open `fener.ino`
- Verify and Upload

Note: If you use a different board, verify external interrupt pins: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

## Hardware overview

- Powertrain: ESCs (rear left/right) at Arduino pins 9/10 (PWM)
- Encoders: 4× wheel encoders on pins 4,5,6,7 (INPUT_PULLUP + interrupts)
- Steering: 2× servos via PCA9685 channels RIGHT=5, LEFT=7 (I2C)
- RC: CH1=A0 (speed), CH2=A1 (angle)
- Jetson Nano: USB serial to Arduino; I2C directly to BNO055

### Example wiring

<p align="center">
  <img src="https://user-images.githubusercontent.com/74321576/146676112-9193da67-8d9f-4756-af2d-0668653eaa6c.JPG" alt="Wiring Example" width="640" />
</p>

## Configuration

Geometry and limits in `consts.h`:

- PulsePerRotate, WheelPerimeter
- FrontWheelMm, FrontRearMm
- Servo PWM limits: `L_L/L_R`, `R_L/R_R`
- Pin map for motors, encoders, RC

PID tuning in `fener.ino`:
- Motors: `PID_left_motor`, `PID_right_motor`
- Steering: `PID_steering`

## Control modes

- RC mode: reads CH1/CH2, smooths angle, normalizes to linear/angular
- Jetson mode: consumes an 8‑byte frame, sets linear/steer targets, returns encoder feedback

Mode selection is by command array index 3; current sketch forces Jetson mode for development.

## Serial protocol (Jetson ↔ Arduino)

- Inbound: 4 packed 16‑bit words; upper 3 bits = command id, lower 12 bits = value
  - 0: speed, 1: angular, 2: BNO055 heading (wrap‑aware), 3: drive mode
- Outbound: 4 bytes = LF, RF, LR, RR encoder byte counters (mod 255 + 1)

## Safety notes

- ESC arming: both motors set to 1000 µs at startup
- Output bounds: PID outputs clamped to safe ranges
- Verify steering limits to avoid mechanical binding

## Related projects

- Fener Vehicle ROS2: https://github.com/sezer-muhammed/Fener-Vehicle-Repo-v2
- Fener Console ROS2: https://github.com/sezer-muhammed/Fener-Console-Repo-v1

## Troubleshooting

- No encoder counts: ensure proper interrupt-capable pins for your board
- Steering inverted: swap servo channels or adjust mapping in `functions.ino`
- Heading jumps: verify BNO055 orientation and update cadence

## Contributing & Feedback

Issues and PRs are welcome. For direct feedback: muhammed.sezer@metu.edu.tr

## Credits

- @sezer-muhammed
- Thanks to @katherinepeterson for readme.so

# Fener Arduino Codes

This repo contains Fener's Arduino codes. 

This code drives the vehicle using encoder data, remote control data, and Jetson Nano commands.

It regularly receives IMU data from Jetson Nano, and sends encoder data to Jetson Nano. It is an extension of the ard_comm node of the ROS2 package that runs on Fener.

## Related Projects

This project is related to these project listed below.

[Fener Vehicle ROS2](https://github.com/sezer-muhammed/Fener-Vehicle-Repo-v2)

[Fener Console ROS2](https://github.com/sezer-muhammed/Fener-Console-Repo-v1)
  
## How To Upload

Clone Repo

```bash
  git clone https://github.com/sezer-muhammed/Fener-Ard-Repo-v1.git
```

Open Arduino IDE and select your port and card type. 

Then Upload the code to Arduino.

### Note
We used Nano Every, so you may need to check digital interrupt pins of your arduino. [Click here to check](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/).

## Requirements

This project uses [PID_V1](https://github.com/br3ttb/Arduino-PID-Library), Servo, [Adafruit_PWMServoDriver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library), and Wire libraries.
First be sure that you installed these libraries.

### Hardware Configuration 

Required connections are listed below. You also can find a graph that shows these connections.

| **Servo Driver** | 
| ----------------------------- |
| Steering Servos to pin **5** and **7** |
| **I2C** pins of the **Arduino** |
| ESC's **5v BEC** Circuit |


| **Arduino** | 
| ----------------------------- |
| ESCs to pin **9** and **10** |
| **I2C** pins of the **Servo Driver** |
| USB Cable of **Jetson Nano** |
| Encoders to pin **4, 5, 6,** and **7** |
| RC Receiver to pin **A0, A1, GND,** and **5v** |

| **Jetson Nano** | 
| ----------------------------- |
| **BNO055** IMU sensor |
| USB Cable of **Arduino** |


### Complete Schematic
![fener_architecture](https://user-images.githubusercontent.com/74321576/146676065-4ae206fa-0116-4e08-8090-edcc8398da87.jpg)

### Example Wiring
![Fener Jetson, ESC, Arduino, Servo sürücü, RC, Pil Tepeden](https://user-images.githubusercontent.com/74321576/146676112-9193da67-8d9f-4756-af2d-0668653eaa6c.JPG)

## Feedback

If you have any feedback please send an email to muhammed.sezer@metu.edu.tr

  
## Contributors

- [@sezer-muhammed](https://www.youtube.com/c/IMSezer)

### Thanks

- [@katherinepeterson](https://www.github.com/octokatherine) For [readme.so](https://readme.so).

 
