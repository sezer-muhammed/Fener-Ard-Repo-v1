
# Fener Arduino Codes

This repo contains Fener's Arduino codes. 

This code drives the vehicle using encoder data, remote control data, and Jetson Nano commands.

It regularly receives IMU data from Jetson Nano, and sends encoder data to Jetson Nano. It is an extension of the ard_comm node of the ROS2 package that runs on Fener.

## Related Projects

This project is related to these project listed below.

[Fener Vehicle ROS2](https://github.com/sezer-muhammed/Fener-Vehicle-Repo-v1)

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



`TODO` Görseller ve tam açıklamalar eklenecek, komponentler tanıtılacak

## Feedback

If you have any feedback please send an email to muhammed.sezer@metu.edu.tr

  
## Contributers

- [@sezer-muhammed](https://www.youtube.com/c/IMSezer)

### Thanks

- [@katherinepeterson](https://www.github.com/octokatherine) For [readme.so](https://readme.so).

  
