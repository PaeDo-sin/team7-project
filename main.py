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


# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.C)

robot = DriveBase(left_motor, right_motor, 55.5, 104)
left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S4)

threshold = 50
kp = 1.2


while True: 
    left_reflection = left_color.reflection()
    right_reflection = right_color.reflection()
    if left_reflection<30 : #30 값은 rgb센서값
        robot.stop()
        break
    else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
        error = right_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, turn_rate)
        wait(10)

robot.straight(20)
wait(100)

now_dir = 1
target_dir = 4

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
a = 2
b = 3
for i in range(a):
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if left_reflection<30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if left_reflection>30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

for i in range(b):
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if right_reflection<30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if right_reflection> 30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

c = 1
for i in range(c):
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if right_reflection < 30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if right_reflection > 30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)

now_dir = 1
target_dir = 4

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

d = 1
for i in range(d):
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if left_reflection<30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if left_reflection>30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)
now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

e = 2
for i in range(e):
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if right_reflection < 30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)
    while True: 
        left_reflection = left_color.reflection()
        right_reflection = right_color.reflection()
        if right_reflection > 30: #30 값은 rgb센서값
            robot.stop()
            break
        else: #에러가 발생했을때 발생한 만큼 턴해줘서 방향 맞춰줌
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
            wait(10)