# linefollower_project
Arduino Nano robot that follows a black line, detects objects with an ultrasonic sensor, and uses a servo gripper to pick up and deliver them. Modular, easy-to-edit code for adapting to similar robots.

What it does:
In its current state, the bot will perform a blind start ( waits for a "flag" to be raised, goes forward from a "parking lot" for a fixed amount of time, then turns left and looks for a black line). Once it finds its line, it will begin following it, while its LED's indicate the state its in (forward, pivots, stops, backing up). Along the course, if it finds an object, it will grab it and drag it along until the end. The end of the course is signaled by a black square. When the robot reaches the end, it will open the gripped, releasing the object on the black square, and then backing up safely.

Uses:
Arduino Nano, 
Servo-controlled gripper, 
8 reflectance sensors, 
Ultrasonic sensor, 
Motors, 
Chasis/wheels (of course).

Software requirements (Libraries):
#include <Arduino.h>, 
#include <Servo.h>, 
#include <Adafruit_NeoPixel.h>, 
#include <SoftwareSerial.h>, 

How to get it running:
Connect the components as shown in the wiring diagram, upload the code to the Arduino Nano, and then connect it to power. Your wiring might differ.
YOU MIGHT HAVE TO ADJUST THE VARIABLES.
-> This is easy to do as all the constants are declared at the begining of the program. It's as simple as changing a number and reuploading the code to the arduino.

You can check the demo on youtube via this link: https://youtu.be/GfSv7V7-erM
