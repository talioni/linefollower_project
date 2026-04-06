# linefollower_project

Arduino Nano robot that follows a black line, detects objects using an ultrasonic sensor, and uses a servo gripper to pick up and deliver them. The code is modular and easy to adapt for similar robotics projects.

## Overview

The robot performs a predefined startup sequence before transitioning into autonomous line-following behavior.

### Behavior

* Starts with a blind sequence:

  * Waits for a start signal ("flag")
  * Moves forward from a starting area for a fixed duration
  * Turns left to begin searching for a line

* Once the line is detected:

  * Follows the black line using reflectance sensors
  * Uses LEDs to indicate current state (forward, pivoting, stopping, reversing)

* Object interaction:

  * Detects objects using an ultrasonic sensor
  * Grabs the object with a servo gripper
  * Drags the object along the course

* End condition:

  * A black square indicates the end of the course
  * Robot releases the object onto the square
  * Moves backward to a safe position

## Hardware Components

* Arduino Nano
* Servo-controlled gripper
* 8 reflectance sensors
* Ultrasonic sensor
* Motors
* Chassis and wheels

## Software Requirements

Required libraries:

```cpp id="lib1"
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
```

## Setup Instructions

1. Connect all components according to the wiring diagram
2. Upload the code to the Arduino Nano
3. Connect the robot to a power source

Note: Wiring may vary depending on your setup.

## Configuration

You may need to adjust variables for your specific hardware.

* All constants are defined at the beginning of the program
* Modify values as needed and re-upload the code

## I/O List

<img width="364" height="734" alt="565903254-858db4d8-c9c4-4cd4-937b-613d0770b5d9" src="https://github.com/user-attachments/assets/c3e806be-5ca6-4f38-a566-f1de7fa2f31d" />

## System Architecture

<img width="1920" height="1080" alt="566188696-ecee1f0c-ba6e-497b-adf3-817160cc7c57" src="https://github.com/user-attachments/assets/3af2517a-cf83-476a-8db5-fb2319e0d93a" />

## Demo

You can view a demonstration of the robot here:
https://youtu.be/GfSv7V7-erM

## Acknowledgements

All hardware was provided by NHL Stenden University of Applied Sciences.
