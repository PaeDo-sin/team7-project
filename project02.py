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


# Write your program here.
ev3.speaker.beep()
grab_motor = Motor(Port.B)
left_motor = Motor(Port.A)
right_motor = Motor(Port.C)



line_sensor = ColorSensor(Port.S1)
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S3)


robot = DriveBase(left_motor, right_motor, wheel_diameter =56, axle_track = 114)
left_color = ColorSensor(Port.S1)
right_color = ColorSensor(Port.S4)

threshold = 50
kp = 1.2

N, E, S, W = 1,2,3,4

def follow_line_one_cell():
    left_reflection = left_color.reflection()
    right_reflection = right_color.reflection()
    if right_reflection < 30: # 0 검은색 100흰색 
        robot.stop()          # 검은색일 때 멈추고   
    else: #검은색이 아닐 때 직진해라 
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(100, turn_rate)
    wait(10)

def left_line_following(speed, kp):
    threshold = 50
    left_reflection = left_color.reflection()
    error = left_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp):
    threshold = 50
    right_reflection = right_color.reflection()
    error = right_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def n_move(n, direction = "right"):
    for _ in range(n):
        if direction == "left":
            while right_sensor.reflection() > 50:
                left_line_following(100, 1.2)
            while right_sensor.reflection() <= 50:
                right_line_following(100, 1.2)
        elif direction == "right":
            while left_sensor.reflection() > 50:
                left_line_following(100, 1.2)
            while right_sensor.reflection() <= 50:
                left_line_following(100, 1.2)
    robot.stop()

def grab_object():
    grab_motor.run_until_stalled(200, then = Stop.COAST, duty_limit =50)

def release_object():
    grab_motor.run_until_stalled(-200, then = Stop.COAST, duty_limit =50)

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) %4
    angle = [0, 90, 180,-90][diff]
    robot.turn(angle)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir) :
    x, y = start_xy
    gx, gy = goal_xy
    dx = gx - x
    dy = gy - y

    if dx != 0:
        target_dir = E if dx > 0 else W
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        for _ in range(steps):
            n_move(1, direction = "right")
            #follow_line_one_cell()
            x += 1 if target_dir == E else -1

    if dy != 0:
        target_dir = N if dy > 0 else S
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
            n_move(1, direction = "left")
            #follow_line_one_cell()
            y += 1 if target_dir ==  N else -1
    return (x,y), now_dir

# start = (1,2)
# goal = (4,1)
# now_dir = E

# (fx, fy), now_dir = move_manhattan(start, goal, now_dir)

# ev3.screen.clear()
# ev3.screen.print("Done:",fx,fy,"dir",now_dir)
# ev3.speaker.beep()

from collections import deque
# .. 로봇이 갈수 있는
# #은 로봇이 갈수 없는
MAP = [
    ".....",
    ".##..",
    ".....",
    ".#.#.",
    "....."
]

G = [[1 if c == "#" else 0 for c in r] for r in MAP]

H, W = len(G), len(G[0])

S, E = (0,0), (4,4) #start,end

def bfs(s,g):
    dist = [[None]*W for _ in range(H)]
    prev = [[None]*W for _ in range(H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0
    while q:
        x, y = q.popleft()
        if (x,y) == g:
            break
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx , ny = x + dx , y + dy
            if 0 <= nx < W and 0 <= ny < H and not G[ny][nx] and dist [ny][nx] is None:
                dist[ny][nx] = dist[y][x]+1
                prev[ny][nx] = (x,y)
                q.append((nx, ny))
    path = []
    if dist[g[1]][g[0]] is not None:
        p = g
        while p :
            path.append(p)
            p = prev[p[1]][p[0]]
        path.reverse()
    return path, dist

def show(path, dist) :
    grid = [['#' if G[y][x] else '.' for x in range(W)] for y in range(H)]
    for x, y in path:
        if (x,y) not in (S,E): 
            grid[y][x] = '*' 
        grid[S[1]][S[0]], grid[E[1]][E[0]] = 'S', 'G'
        print("=== 경로 맵===")
        for r in grid: print(''.join(r)) 
        print("\n=== 거리 히트맵===")
        for r in dist: print(' '.join(' .' if d is None else "%2d" % d for d in r)) 
        print("\n경로:", path)
        print("길이:", len(path)-1 if path else "없음")

if __name__ == "__main__":

    path, dist = bfs(S,E)
    print("===현재 MAP 출력===")
    for r in MAP:
        print(r)
    print()
    show(path, dist)

robot.straight(100)
n_move(1, direction = "left")
while ultra_sensor.distance() > 30:
    left_line_following(100, 1.2)
robot.stop()


grab_object()
wait(1000)

object_color = object_detector.color()
print("Detected object colot : ", object_color)

if object_color == Color.RED:
    robot.turn(180)
    n_move(1, direction = "right")
    robot.straight(50)
    robot.turn(-90)

    n_move(1, direction = "left")
    robot.turn(90)
    n_move(1, direction = "left")
    release_object()
elif object_color == Color.BLUE:
    robot.turn(180)
    n_move(1, direction = "right")
    robot.turn(-90)
    n_move(2, direction = "left")
    robot.turn(90)
    n_move(1, direction = "left")
    release_object()
