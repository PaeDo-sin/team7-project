#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase

# --- 1. 설정 및 초기화 (Configuration) ---
ev3 = EV3Brick()

# 모터 설정
left_motor = Motor(Port.A)
right_motor = Motor(Port.C)
grab_motor = Motor(Port.B)

# 센서 설정
left_color_sensor = ColorSensor(Port.S1)   # 라인 트레이싱 (좌)
right_color_sensor = ColorSensor(Port.S4)  # 라인 트레이싱 (우)
object_sensor = ColorSensor(Port.S2)       # 물체 색상 판별
ultra_sensor = UltrasonicSensor(Port.S3)   # 거리 감지

# 로봇 드라이브 베이스 설정 (바퀴 지름, 축간 거리 확인 필요)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# 상수 정의
SPEED_NORMAL = 150
SPEED_FAST = 200
SPEED_SLOW = 80
BLACK_THRESHOLD = 25  # 검은색 라인 기준값 (환경에 맞춰 조정)
WHITE_THRESHOLD = 60
KP = 1.2             # 라인 트레이싱 비례 제어 상수

# 방향 상수 (N을 0으로 기준)
# N: 이미지상 오른쪽(진입 방향)
# E: 이미지상 아래쪽
# S: 이미지상 왼쪽(Start/Zone 방향)
# W: 이미지상 위쪽
DIR_N = 0  
DIR_E = 1  
DIR_S = 2  
DIR_W = 3  

# 맵 크기 (가로 4칸, 세로 3칸으로 가정 - 이미지 점 1~12 기준)
MAP_WIDTH = 4
MAP_HEIGHT = 3

# 전역 변수: 로봇의 현재 상태
current_x = 0
current_y = 0
current_dir = DIR_N  # 시작 시 N 방향을 보고 있다고 가정

# --- 2. 기본 동작 함수 (Basic Movements) ---

def line_follow_until_intersection(speed=SPEED_NORMAL):
    """
    교차로(양쪽 센서 모두 검은색)를 만날 때까지 라인 트레이싱을 하며 전진합니다.
    """
    while True:
        left_val = left_color_sensor.reflection()
        right_val = right_color_sensor.reflection()

        # 교차로 감지 조건: 양쪽 다 검은색일 때
        if left_val < BLACK_THRESHOLD + 10 and right_val < BLACK_THRESHOLD + 10:
            robot.stop()
            ev3.speaker.beep(frequency=500, duration=50)
            break
        
        # 장애물 감지 (이동 중 감지)
        if ultra_sensor.distance() < 50: # 50mm (5cm) 이내 감지
            robot.stop()
            return "OBSTACLE"

        # PID 라인 트레이싱 (왼쪽 센서 기준 라인 타기)
        error = left_val - ((BLACK_THRESHOLD + WHITE_THRESHOLD) / 2)
        turn_rate = KP * error
        robot.drive(speed, turn_rate)
    
    # 교차로를 살짝 통과하여 중심을 맞춤
    robot.straight(30)
    return "ARRIVED"

def turn_robot(target_dir):
    """
    현재 방향(current_dir)에서 목표 방향(target_dir)으로 회전합니다.
    """
    global current_dir
    
    diff = (target_dir - current_dir) % 4
    
    if diff == 1:   # 오른쪽 90도
        robot.turn(90)
    elif diff == 2: # 뒤로 180도
        robot.turn(180)
    elif diff == 3: # 왼쪽 90도 (혹은 -90)
        robot.turn(-90)
    
    # 회전 후 방향 업데이트
    current_dir = target_dir

def move_one_block(direction):
    """
    한 블록(교차로에서 다음 교차로) 이동
    """
    turn_robot(direction)
    result = line_follow_until_intersection()
    return result

def grab_object_action():
    """집게로 물체를 잡습니다."""
    ev3.speaker.say("Grab")
    # 물체를 확실히 잡기 위해 살짝 전진
    robot.straight(20) 
    grab_motor.run_until_stalled(200, then=Stop.HOLD, duty_limit=50)

def release_object_action():
    """집게를 풉니다."""
    ev3.speaker.say("Release")
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def recover_to_previous_node():
    """
    이동 중 장애물을 만나 멈췄을 때, 180도 회전하여 
    방금 출발했던 교차로(current_x, current_y)로 되돌아옵니다.
    """
    global current_dir
    ev3.speaker.say("Back")
    
    # 1. 180도 회전
    robot.turn(180)
    
    # 2. 방향 정보 업데이트 (뒤로 돌았으므로 반대 방향이 됨)
    current_dir = (current_dir + 2) % 4
    
    # 3. 라인 트레이싱으로 이전 교차로까지 복귀
    line_follow_until_intersection()
    
    # 4. 교차로 중앙 정렬은 line_follow 함수 내에서 수행됨

# --- 3. 고수준 내비게이션 (High-level Navigation) ---

def move_to_coordinate(target_x, target_y):
    """
    Manhattan 거리 계산을 통해 목표 좌표로 이동합니다.
    이동 중 장애물을 만나면 True를 반환합니다.
    """
    global current_x, current_y
    
    print("Moving to ({}, {})".format(target_x, target_y))

    # X축 이동
    while current_x != target_x:
        direction = DIR_N if target_x > current_x else DIR_S
        status = move_one_block(direction)
        
        if status == "OBSTACLE":
            return True # 장애물 발견
            
        current_x += 1 if direction == DIR_N else -1
    
    # Y축 이동
    while current_y != target_y:
        direction = DIR_E if target_y > current_y else DIR_W
        status = move_one_block(direction)
        
        if status == "OBSTACLE":
            return True # 장애물 발견

        current_y += 1 if direction == DIR_E else -1
        
    return False # 장애물 없이 도착

def deliver_to_zone(color_detected):
    """
    (중요) 현재 위치에서 출발하여 Red/Blue 존까지 한번에 이동합니다.
    경로: 현재위치 -> (0,0) -> 왼쪽 회전(Start방향) -> 왼쪽 회전(Zone방향) -> 배달
    """
    global current_dir, current_x, current_y
    
    ev3.speaker.say("Delivery")
    
    # 1. 일단 그리드의 출입구인 (0,0)까지 이동
    #    (사용자 요청: 0,0으로 가는게 목적이 아니라 Zone으로 가는 과정임)
    move_to_coordinate(0, 0)
    
    # 2. (0,0)에서 Start/Zone 방향(서쪽/왼쪽)을 바라봄
    turn_robot(DIR_S) 
    
    # 3. 메인 통로로 진출 (Start 영역 근처까지 이동)
    #    거리값(200)은 맵 실측에 따라 조정 필요
    robot.straight(200) 
    
    # 4. 이제 색상 구역으로 진입하기 위해 왼쪽(남쪽)으로 회전
    #    Start 구역 옆에 Red/Blue 존이 세로로 배치되어 있다고 가정
    robot.turn(-90) 
    # 방향 변수 업데이트 (수동 조작시에도 업데이트 해주는 것이 안전)
    current_dir = DIR_E 
    
    # 5. 색상별 거리 이동 및 배달
    if color_detected == Color.RED:
        ev3.speaker.say("Red")
        robot.straight(100) # 빨간 구역까지의 거리
        release_object_action()
        # 복귀: 뒤로 물러나기
        robot.straight(-100)

    elif color_detected == Color.BLUE:
        ev3.speaker.say("Blue")
        robot.straight(300) # 파란 구역은 더 멀리 있음
        release_object_action()
        # 복귀: 뒤로 물러나기
        robot.straight(-300)
    
    else:
        # 색상 인식 실패 시 안전하게 놓고 복귀
        ev3.speaker.beep()
        release_object_action()
    
    # 6. 다시 그리드 탐색을 위해 초기 위치(0,0) 상태로 복귀
    #    현재 로봇은 Start 영역에서 남쪽(E)을 보고 있음
    robot.turn(90)  # 다시 Start 방향(S)의 반대인 N(오른쪽)을 봄? 
                    # 아까 왼쪽(-90) 돌았으니 반대로(90) 돌면 다시 Start 방향을 보게 됨 (X)
                    # 아까 S방향 보고 오다가 왼쪽(-90) 돌아서 E방향이 됨.
                    # 다시 (0,0)으로 가려면: 뒤로(N방향) 가거나, 제자리 턴해서 가야 함.
    
    # 가장 깔끔한 복귀: 
    # 배달 후 robot.straight(-)로 다시 메인 통로(Start 앞)로 왔음.
    # 현재 방향: E (아래쪽)
    # (0,0)은 로봇의 왼쪽(동쪽, N방향)에 있음. (이미지 기준)
    
    robot.turn(-90) # 왼쪽으로 회전 -> 이제 동쪽(N방향, 그리드 입구)을 봄
    robot.straight(200) # 아까 나왔던 만큼 다시 들어감 -> (0,0) 도착
    
    # 상태 재설정
    current_x = 0
    current_y = 0
    current_dir = DIR_N # (0,0)에서 그리드 안쪽(N)을 바라보는 상태

# --- 4. 메인 로직 (Main Logic) ---

def main():
    global current_x, current_y
    
    # 1. 초기화: 집게 벌리기
    release_object_action()
    ev3.speaker.say("Ready")
    
    # 2. 탐색 경로 생성 (Snake Pattern)
    search_path = []
    for y in range(MAP_HEIGHT):
        if y % 2 == 0:
            xs = range(MAP_WIDTH)
        else:
            xs = range(MAP_WIDTH - 1, -1, -1)
        for x in xs:
            search_path.append((x, y))
            
    # 3. 탐색 루프
    path_index = 0
    while path_index < len(search_path):
        target = search_path[path_index]
        
        # 현재 목표 좌표로 이동 시도
        found_obstacle = move_to_coordinate(target[0], target[1])
        
        if found_obstacle:
            # [상황 A] 이동 중에 장애물 발견 (길 중간)
            ev3.speaker.beep()
            
            # 1. 잡기
            grab_object_action()
            wait(500)
            
            # 2. 색상 확인
            detected_color = object_sensor.color()
            
            # 3. [핵심] 다시 이전 교차로로 되돌아가기 (좌표 보정)
            recover_to_previous_node()
            
            # 4. 배달 (Red/Blue Zone으로)
            deliver_to_zone(detected_color)
            
            # 5. 배달 후에는 (0,0)에 있으므로, 현재 목표였던 곳부터 다시 탐색 재개
            continue 
            
        else:
            # [상황 B] 좌표 도착 (장애물 없이)
            # 도착한 노드 위치에 물체가 있는지 확인
            if ultra_sensor.distance() < 50:
                 ev3.speaker.beep()
                 
                 # 1. 잡기 (이미 노드 위이므로 straight 불필요하거나 조금만)
                 grab_object_action()
                 detected_color = object_sensor.color()
                 
                 # 2. 배달 (이미 노드 위이므로 recover 불필요)
                 deliver_to_zone(detected_color)
                 continue

        # 다음 좌표로 목표 설정
        path_index += 1

    ev3.speaker.say("All Done")
    ev3.speaker.beep(500, 1000)

if __name__ == "__main__":
    main()