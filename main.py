#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

def deg_to_rad(y):
    return(y*(math.pi/180))

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

def update_position(x, y, theta, ur, ul, t, l=14.8, r=3):
    vr = ur * r
    vl = ul * r

    if (vr - vl == 0):
        newx = x + vr*math.cos(theta)*t
        newy = y + vl*math.sin(theta)*t
        return newx, newy, theta

    radius = (l/2) * (vr+vl)/(vr-vl)
    iccx = x - radius*math.sin(theta)
    iccy = y + radius*math.cos(theta)
    omega = (vr-vl)/l
    alpha = omega * t

    newx = math.cos(alpha)*(x-iccx) - math.sin(alpha)*(y-iccy) + iccx
    newy = math.sin(alpha)*(x-iccx) + math.cos(alpha)*(y-iccy) + iccy
    newTheta = theta + alpha

    while (newTheta > 2*math.pi):
        newTheta = newTheta - 2*math.pi

    while (newTheta < 0):
        newTheta = newTheta + 2*math.pi

    return newx, newy, newTheta


ev3 = EV3Brick()

# Initialization of Motors and Sensors

leftMotor = Motor(port = Port.B, positive_direction = Direction.COUNTERCLOCKWISE)
rightMotor = Motor(port = Port.C, positive_direction = Direction.COUNTERCLOCKWISE)
leftBump = TouchSensor(port = Port.S3)
rightBump = TouchSensor(port = Port.S1)
us = UltrasonicSensor(port = Port.S2)

# Wait until center button press to move

wait_for_button()


run_motors(leftMotor, -180, rightMotor, -180)
ev3.speaker.beep()

while (not (leftBump.pressed() or rightBump.pressed())):
    pass

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)

wait(500)

# Turn to face the wall
x = 0
y = 0
theta = math.pi/2
run_motors(leftMotor, -150, rightMotor, 150)
wait(1500)

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)

x, y, theta = update_position(x, y, theta, deg_to_rad(-150), deg_to_rad(150), 1.5)
print("x", str(x))
print("y", str(y))
print("theta", str(theta))

ev3.speaker.beep()

wait(1500)
dist = us.distance()


# Wall following
ideal = 175
k = 0.5

prev_error = 0

time_passed = 0

stop_watch = StopWatch()

j = 0
num_left_turns = 0
left_count = 0

while((time_passed < 20) or ((not (0 < x < 15 + 7 * num_left_turns)) or (not (-30 < y < 60)))):

    stop_watch.resume()

    if(leftBump.pressed() or rightBump.pressed()):
        print("x:", str(x))
        print("y:", str(y))
        print("theta:", str(theta))       
        stop_watch.pause()
        stop_watch.reset()
        run_motors(leftMotor, -80, rightMotor, 200)
        wait(1500)
        stop_motors(leftMotor, rightMotor)
        x, y, theta = update_position(x, y, theta, deg_to_rad(200), deg_to_rad(-80), 1.5)

        print("x:", str(x))
        print("y:", str(y))
        print("theta:", str(theta))
        continue

    dist = us.distance()

    error = dist-ideal
    error = min(error, 80)

    ul = k * error - 150
    ur = -k * error - 150
    run_motors(leftMotor, ul, rightMotor, ur)

    prev_error = error
    wait(50)

    stop_watch.pause()
    
    #stop_motors(leftMotor, rightMotor)
    t = stop_watch.time() / 1000

    x, y, theta = update_position(x, y, theta, deg_to_rad(ur), deg_to_rad(ul), t)
    time_passed = time_passed + 1

    if (j % 5 == 0):
        print("x:", str(x))
        print("y:", str(y))
        print("theta:", str(theta))

    if (ur < ul):
        left_count += 1
    else:
        if (left_count > 20):
            num_left_turns += 1
            ev3.speaker.beep()
        left_count = 0

    stop_watch.reset()
    j += 1
    #wait(50)

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
wait(10)
stop_motors(leftMotor, rightMotor)
ev3.speaker.beep()

run_motors(leftMotor, 150, rightMotor, -150)
wait(1500)

stop_motors(leftMotor, rightMotor)
wait(500)

run_motors(leftMotor, -180, rightMotor, -180)

while (not (leftBump.pressed() or rightBump.pressed())):
    pass

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)

run_motors(leftMotor, 180, rightMotor, 180)
wait(3000)

stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)
stop_motors(leftMotor, rightMotor)

ev3.speaker.beep()