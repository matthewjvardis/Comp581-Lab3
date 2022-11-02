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


ev3 = EV3Brick()

# Initialization of Motors and Sensors

leftMotor = Motor(port = Port.B, positive_direction = Direction.CLOCKWISE)
rightMotor = Motor(port = Port.C, positive_direction = Direction.CLOCKWISE)
leftBump = TouchSensor(port = Port.S2)
rightBump = TouchSensor(port = Port.S1)
us = UltrasonicSensor(port = Port.S4)
gyro = GyroSensor(port = Port.S3)

# Wait until center button press to move

print(gyro.angle())
wait_for_button()


run_motors(leftMotor, 180, rightMotor, 180)
ev3.speaker.beep()

while (not (leftBump.pressed() or rightBump.pressed())):
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
ideal = 200
k = 0.5

prev_error = 0

while(True):

    if(leftBump.pressed() or rightBump.pressed()):
        run_motors(leftMotor, 80, rightMotor, -200)
        wait(1500)

    dist = us.distance()

    if (dist == 2550 and prev_error < 0):
        stop_motors(leftMotor, rightMotor)
        wait(200)
        run_motors(leftMotor, -200, rightMotor, -200)
        wait(1000)
        stop_motors(leftMotor, rightMotor)
        wait(200)

    error = dist-ideal
    error = min(error, 80)

    ul = -k * error + 150
    ur = k * error + 150
    run_motors(leftMotor, ul, rightMotor, ur)

    prev_error = error
    wait(500)

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
wait(10)
stop_motors(leftMotor, rightMotor)

ev3.speaker.beep()