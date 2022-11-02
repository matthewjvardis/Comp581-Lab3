#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def deg_to_rad(y):
    return(y*(3.14159265358979/180))

def run_motors(leftMotor, leftSpeed, rightMotor, rightSpeed):
    leftMotor.run(leftSpeed)
    rightMotor.run(rightSpeed)

def stop_motors(leftMotor, rightMotor):
    leftMotor.run_time(0,0,then=Stop.BRAKE, wait = False)
    rightMotor.run_time(0,0,then=Stop.BRAKE, wait = False)

def wait_for_button():
    pressed = 0
    while (Button.CENTER not in EV3Brick.buttons.pressed()):
        pass

def play_music(ev3):
    ev3.speaker.play_notes(["D3/8", "D3/8", "D4/8"], 225)
    wait(100)
    ev3.speaker.play_notes(["A3/8"], 150)
    wait(200)
    ev3.speaker.play_notes(["G#3/8"], 150)
    wait(50)
    ev3.speaker.play_notes(["G3/8", "F3/8"], 150)
    ev3.speaker.play_notes(["D3/8", "F3/8", "G3/8"], 250)


ev3 = EV3Brick()

# Initialization of Motors and Sensors

leftMotor = Motor(port = Port.B, positive_direction = Direction.CLOCKWISE)
rightMotor = Motor(port = Port.C, positive_direction = Direction.CLOCKWISE)
bump = TouchSensor(port = Port.S2)
us = UltrasonicSensor(port = Port.S4)

# Wait until center button press to move

wait_for_button()

run_motors(leftMotor, 180, rightMotor, 180)
ev3.speaker.beep()

while (not bump.pressed()):
    pass

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)

wait(500)

# Turn to face the wall
run_motors(leftMotor, -50, rightMotor, -150)
wait(3200)

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
ev3.speaker.beep()


# Wall following
ideal = 250
k = 2

while(True):
    dist = us.distance()

    if(bump.pressed()):
        run_motors(leftMotor, 80, rightMotor, -200)
        wait(1500)

    ul = -k * (dist-ideal)
    ur = k * (dist-ideal)
    run_motors(leftMotor, ul, rightMotor, ur)
    wait(100)
    run_motors(leftMotor, 150, rightMotor, 150)
    wait(300)
    


stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
wait(10)
stop_motors(leftMotor, rightMotor)

play_music(ev3)
