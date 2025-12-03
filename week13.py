#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color, SoundFile
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# -----------------------------
# 1. 초기화 및 설정
# -----------------------------
ev3 = EV3Brick()

# 모터 (좌/우/집게)
left_motor = Motor(Port.A)
right_motor = Motor(Port.C)
grab_motor = Motor(Port.B)

# 센서
left_sensor  = ColorSensor(Port.S1)      # 왼쪽
right_sensor = ColorSensor(Port.S4)      # 오른쪽
object_sensor = ColorSensor(Port.S2)     # 가운데 (물체 색상)
ultra_sensor = UltrasonicSensor(Port.S3) # 장애물 감지

# 로봇 구동 베이스
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# -----------------------------
# 2. 파라미터 설정 (단순화 및 안정화)
# -----------------------------
DRIVE_SPEED = 200 # [수정] 속도 증가 (150 -> 200)
TURN_SPEED  = 80
robot.settings(straight_speed=DRIVE_SPEED, turn_rate=TURN_SPEED)

KP = 0.8
BLACK_LIMIT   = 65      # 교차로 검은색 인식 기준
FOLLOW_TARGET = 50      # 라인 팔로잉 목표값 (회색)

# [수정] 장애물 감지 거리 증가 (250mm -> 350mm)
OBSTACLE_DIST = 350

# 방향 정의 (N:북(-Y), E:동(+X), S:남(+Y), W:서(-X))
DIR_N, DIR_E, DIR_S, DIR_W = 0, 1, 2, 3

# 현재 상태
current_x   = 0
current_y   = 0
current_dir = DIR_E

# 거리 설정 (분리 유지)
START_DIST = 500       # 시작/종료 시 이동 거리
ZONE_ENTRY_DIST = 200  # 배달 구역 진입 거리

# -----------------------------
# 3. 방향 전환
# -----------------------------

def turn_to_direction(target_dir):
    """현재 방향에서 target_dir 쪽으로 회전"""
    global current_dir

    diff = (target_dir - current_dir) % 4

    if diff == 0:
        return            
    elif diff == 1:
        robot.turn(90)    
    elif diff == 2:
        robot.turn(180)   # 180도 턴
    elif diff == 3:
        robot.turn(-90)   

    robot.stop()
    wait(100)
    
    current_dir = target_dir

# -----------------------------
# 4. 라인 한 칸 이동 (요청하신 센서 로직 적용)
# -----------------------------
def line_follow_one_block(direction):
    """
    [핵심 로직] 방향별 센서 역할 분담 적용
    - E, S: 왼쪽(Trace), 오른쪽(Stop)
    - W, N: 오른쪽(Trace), 왼쪽(Stop)
    """

    robot.reset()
    robot.straight(50)   # 교차로 탈출 거리

    intersection_count = 0

    while True:
        # 1) 안전상 거리 제한
        if robot.distance() > 450:
            robot.stop()
            break

        left_val  = left_sensor.reflection()
        right_val = right_sensor.reflection()

        # [방향에 따른 센서 역할 분담 로직]
        if direction in (DIR_E, DIR_S):
            follow_val = left_val
            inter_val  = right_val
            error      = follow_val - FOLLOW_TARGET
            print(inter_val)
            
        else:
            follow_val = right_val
            inter_val  = left_sensor.reflection() # 왼쪽 센서 (Stop)
            error      = -(follow_val - FOLLOW_TARGET) # 부호 반전

        # [교차로 감지] 연속 2회 검은색 감지 시 정지
        if inter_val < BLACK_LIMIT:
            intersection_count += 1
        else:
            intersection_count = 0
        
        if intersection_count >= 2:
            robot.stop()
            break

        # [장애물 감지]
        if ultra_sensor.distance() < OBSTACLE_DIST:
            robot.stop()
            return "OBSTACLE"

        # 주행
        turn_rate = KP * error
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(10)

    # 교차로 중앙 정렬
    robot.straight(60)
    return "ARRIVED"

def move_to_grid(tx, ty):
    """좌표 기반 이동"""
    global current_x, current_y

    # X축 이동
    while current_x != tx:
        direction = DIR_E if tx > current_x else DIR_W
        turn_to_direction(direction)
        result = line_follow_one_block(direction)
        if result == "OBSTACLE": return True
        current_x += 1 if direction == DIR_E else -1

    # Y축 이동
    while current_y != ty:
        direction = DIR_S if ty > current_y else DIR_N
        turn_to_direction(direction)
        result = line_follow_one_block(direction)
        if result == "OBSTACLE": return True
        current_y += 1 if direction == DIR_S else -1

    return False

# -----------------------------
# 5. 잡기 및 배달 로직
# -----------------------------

def grab_and_check_color():
    ev3.speaker.beep()
    
    # 1. 색상 인식을 위해 접근 (350mm 감지 후 250mm 직진)
    APPROACH_DIST = OBSTACLE_DIST - 100 
    
    # 접근 직전에 속도를 낮춤
     
    
    robot.straight(APPROACH_DIST) # 250mm 접근
    wait(200)
    
    # 2. 잡기 위해 마지막 70mm 접근
    robot.straight(70)
    # 3. [핵심 수정] 잡기 (색상 인식보다 먼저)
    grab_motor.run_until_stalled(200, then=Stop.HOLD, duty_limit=50)
    wait(500)
    
    # 4. 색상 인식 (물체를 잡은 상태에서 안정적으로 인식)
    detected_color = object_sensor.color()
    # 작업 후 원래 속도로 복구
    
    if detected_color == Color.RED:
        print("!!! RED !!!")
        ev3.speaker.say("Red")
    elif detected_color == Color.BLUE:
        print("!!! BLUE !!!")
        ev3.speaker.say("Blue")
    else:
        print("Unknown")
        ev3.speaker.beep(500, 200)
    robot.straight(130)
    return detected_color

def release_object():
    ev3.speaker.beep(600, 100)
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def recover_and_deliver(color):
    """
    180도 회전 -> 복귀 -> 배달 -> 원위치
    """
    global current_x, current_y, current_dir
    
    print("current_pose", current_x,current_y, current_dir)
    # 1. 뒤로 돌아 (Recover)
    robot.turn(170)
    wait(200)
    current_dir = (current_dir + 2) % 4
    print(current_dir)

    # 2. 왔던 길 되돌아가기 (교차로 복귀)
    line_follow_one_block(current_dir)
   
    # 3. 타겟 설정
    target_y = 0
    if color == Color.RED: target_y = 1
    elif color == Color.BLUE: target_y = 2
    
    print("Deliver to (0, {})".format(target_y))
    
    # 4. 배달 구역 입구로 이동
    move_to_grid(0, target_y)
    
    # 5. 배달 (서쪽 방향)
    turn_to_direction(DIR_W)
    
    # 짧은 거리(200mm)만 진입
    robot.straight(ZONE_ENTRY_DIST)
    release_object()
    robot.straight(-ZONE_ENTRY_DIST)
    
    # 6. 복귀 (동쪽 방향)
    turn_to_direction(DIR_E)
    
    current_x = 0
    current_y = target_y

def finish_mission():
    ev3.speaker.say("Finish")
    move_to_grid(0, 0)
    turn_to_direction(DIR_W)
    
    # 종료 시에는 긴 거리(500mm) 복귀
    robot.straight(START_DIST)
    
    ev3.speaker.play_file(SoundFile.CHEERING)

# -----------------------------
# 6. 메인 프로그램
# -----------------------------
def initial_straight_with_obstacle_check():
    """
    [핵심 추가] 시작 시 500mm 구간을 장애물 감지하며 주행합니다.
    """
    global current_x, current_y
    robot.reset()
    
    print("Initial 500mm run started...")
    
    # 500mm 직진 (START_DIST)하며 장애물 감지
    robot.drive(DRIVE_SPEED, 0)
    
    while robot.distance() < START_DIST:
        if ultra_sensor.distance() < OBSTACLE_DIST:
            robot.stop()
            print("Obstacle found on initial path!")
            
            # 장애물 처리 루틴 시작
            color = grab_and_check_color()
            recover_and_deliver(color)
            
            # 처리 후 로봇은 (0, y)에 있으므로, (0, 0)으로 이동하여 그리드 시작
            move_to_grid(0, 0)
            return

        wait(10)
        
    robot.stop()
    print("Initial 500mm run completed. Starting grid search.")
    
    # 500mm 주행 후, 로봇은 그리드 시작점 (0,0)에 도달
    current_x = 0
    current_y = 0
    # 방향은 처음부터 동쪽(DIR_E)으로 설정되어 있음
    

def main():
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)
    ev3.speaker.beep()

    # [수정] 500mm 초기 진입 시 장애물 감지 기능 추가
    initial_straight_with_obstacle_check()

    # 맵 좌표
    POINTS = {
        "1": (2, 0), "5": (2, 1), "9": (2, 2),
        "10": (3, 2), "6": (3, 1), "2": (3, 0),
        "3": (4, 0), "7": (4, 1), "11": (4, 2)
    }
    
    # 방문 순서 (현재 0,0에 있으므로 1번 지점부터 탐색 시작)
    visit_order = ["1", "5", "9", "10", "6", "2", "3", "7", "11"]

    for name in visit_order:
        tx, ty = POINTS[name]
        
        found_obstacle = move_to_grid(tx, ty)
        
        if found_obstacle:
            ev3.speaker.beep()
            wait(100)
            
            color = grab_and_check_color()
            print(color)
            recover_and_deliver(color)
            
            # 배달 후 원래 가려던 목표로 다시 이동
            print("Restarting search to ({}, {})".format(tx, ty))
            move_to_grid(tx, ty)

    finish_mission()

if __name__ == "__main__":
    main()