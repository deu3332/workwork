#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy   
import math
from sensor_msgs.msg import LaserScan      # LaserScan -> ranges 라는 필드에 배열로된 거리정보 있음. 카메라 기준으로 왼쪽부터 오른쪽 까지 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry          # odometry -> 주행 속도정보 측정 


class Stack(list):  # 좌표저장을 위한 스택
    push = list.append                # push 변수에 매개변수로 받아온 list의 값 추가 
    run_count = 0                     # run_count 0으로 초기화 

    def check_data(self):               # 이거 왜있는지 모르겠음 아는사람 
        if not self:
            return True
        else:
            return False

    def check_end(self):
        return self[-1]


class MazeRun:  # 메이즈 1차 탈출 클래스
    stackX = Stack()   # 좌표 x,y값 스택에 저장
    stackY = Stack()
    abs_poseX = 0  # 주어진 맵  x,y 좌표 
    abs_poseY = 0

    def __init__(self):               # __init__(self) 메소드 : 초기화 메소드라고 생각하면 편함. 인자는 self로 고정 
        self.driving_forward = True       # 직진 상태 여부 
        self.rotation = False             # 회전 상태 여부 
        self.enter = True                 # 탈출 상태 여부 
        self.turn = Twist()
        self.twist = Twist()
        self.run_count = 0
        self.rate = rospy.Rate(10)          # 토픽 발행 속도 초당 10

        self.range_ahead = 1
        self.range_left = 1
        self.range_right = 1

        self.poseX = 0   # odom은 위치와 속도의 추정치 공간으로 보면됨. 그러니까 로봇이 가면서 실시간으로 위치를 변화시키는데 그 
        self.poseY = 0   # 위치를 저장하면서 실시간으로 위치를 갱신한다는 개념? 

        # 토픽 발행
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)       # odom 메세지 토픽 구독자 발행 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)         # cmd_vel  토픽 발행 Twist 메세지 타입  
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)          # scan 메세지토픽 구독자 발행 

    # 스캔 콜백 함수 -> 노드가 토픽 구동하면 발생 
    def scan_callback(self, msg):     # msg가 도착할때 마다 메세지를 매개변수로 하여 callback 함수는 호출됨. 
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]                     # 카메라 기준으로 왼쪽부터 오른쪽 까지 이므로 2로 나눠서 카메라로 부터 정면까지의 거리  
        self.range_left = msg.ranges[int(len(msg.ranges) / 1.3)]  # 1350       # 왼쪽 각도 
        self.range_right = msg.ranges[len(msg.ranges) / 4]  # 450              # 오른쪽 각도 

        
    # 오돔 좌표와 지도의 좌표를 계산, 오돔 콜백 함수
    def odom_callback(self, msg):          # 로봇 이동시 마다 지도의 좌표 및 odom임시 저장소를 이용해서 좌표를 계산하는 메소드. 
        self.poseX = msg.pose.pose.position.x       # 이동할때마다의 x 좌표 
        self.poseY = msg.pose.pose.position.y       #       //      y 좌표 
        self.abs_poseX = self.poseX - 7.92          # 위에서 한거 좌표값 받아서 x,y 좌표 갱신, 7.92 / 5.30은 출발지의 좌표값이라서 빼주고 더함.  
        self.abs_poseY = self.poseY + 5.30          

        
    # 계산한 좌표 값을 스택에 저장
    def save_pose(self):       # 1차 탈출시 같은 경로로 돌아가서 목적지에 도착하게끔 하기 위한 경로 저장 메소드 
        if self.rotation is True:   # 회전을 할 때 마다 저장함 
            self.stackX.push(self.abs_poseX)          # x / y 좌표값 스택에 저장 
            self.stackY.push(self.abs_poseY)
            self.run_count = self.run_count + 1  # 카운트로 몇 번 쌓이는지 확인
            Stack.run_count = Stack.run_count + 1

    # 미로 탈출 함수
    def maze_escape(self):
        if self.enter is True:  # 탈출 시작 알림
            for i in range(13):
                start_maze = Twist()                      # Twist 메세지 타입 start_maze 객체 
                start_maze.angular.z = -math.radians(90)  # 이동전에 90도로 틀고 진행 
                self.cmd_vel_pub.publish(start_maze)      # 틀고나서 cmd_vel_pub 토픽 발행  
                self.rate.sleep()                         # 잠깐 대기 
            self.enter = False                            # 처음 시작 시 90도 회전 후 시작 플래그 False
        else:
            if self.rotation is True:  ## 회전하는 경우
                for i in range(10):
                    self.cmd_vel_pub.publish(self.turn) # 회전 토픽 발행 
                    self.rate.sleep()                   # 잠깐 대기 
                self.rotation = False                   # 회전 다하고 플래그 false 
            else:                      ## 회전 안하는 경우 
                if self.driving_forward is True:         # 직진으로 주행 중인 상태
                    if self.range_ahead < 0.7:           # 정면 물체 인식 거리가 0.7 보다 작으면
                        self.driving_forward = False     # 직선 주행 정지
                else:                  ## 회전도 안하고 직진으로 주행도 아닐때 -> 계속 직진하라는 상태 
                    if self.range_ahead > 0.7:           # 정면 물체 인식 거리가 0.7 보다 크면
                        self.driving_forward = True      # 직선 주행 계속
                twist = Twist()
                if self.driving_forward is True:  ## 직선 주행 중일 경우
                    twist.linear.x = 1
                    if self.range_left > self.range_right:           # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 멀 경우
                        self.turn.angular.z = math.radians(90)       # 왼쪽으로 90도 회전하라
                    elif self.range_left < self.range_right:         # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 가까울 경우
                        self.turn.angular.z = -math.radians(90)       # 오른쪽으로 90도 회전하라
                else:  # 직선 주행이 아닐경우 회전
                    self.rotation = True                              # 회전 해야하기 위해서 회전 플래그 True 
                self.cmd_vel_pub.publish(twist)                        # twist cmd_vel_pub 발행 
                self.rate.sleep()               


# 1차 탈출, 2차 탈출 시작 조건을 위한 클래스
class Begin:
    def __init__(self):       # 초기화 메소드 
        self.mr = MazeRun()  # 1차 탈출 클래스 객체 선언
        self.tb = TakeBack()  # 2차 탈출 클래스 객체 선언
        self.target = False  # 도착 상태 플래그 

    # 출발, 도착 좌표 지점 인식 함수
    # 출발지 좌표 = (-7.92, 5.3)
    # 목적지 좌표 = (7.95, -5.15)
    
    def get_target(self):                           # 목적지를 발견 여부 확인하는 메소드  
        while not rospy.is_shutdown():              # 종료 전까지 무한루프 
            
            if (-8.55 < self.mr.abs_poseX < -7.55) and (
                    5.05 < self.mr.abs_poseY < 5.55) and self.target is True:  
                # 도착 상태 플래그가 true이고 목적지 좌표 값에 들어가면 멈춤 
                break

            if ((7.75 < self.mr.abs_poseX < 8.15) and (
                    -5.35 < self.mr.abs_poseY < -4.95)) or self.target is True:  
                # 도착 상태 플래그가 true이고 도착지 좌표값에 둘어가면  
                self.target = True    # 도착 상태 플래그 True           
                self.tb.re_escape()   # 2차 탈출 메소드 불러옴 
            else:
                self.mr.maze_escape()  # 아닐 경우 1차 탈출메소드 불러옴 
                self.mr.save_pose()    # 좌표 값 저장

# 2차 탈출 클래스 
class TakeBack:
    def __init__(self):          # 2차 탈출에 사용할 각종 변수들 초기화 
        self.mr = MazeRun()

        self.return_start = True  # 2차 탈출 시작 플래그 
        self.go_back = True       # 직선 주행 플래그 
        self.re_rotation = False  # 회전 상태 플래그 
        self.rate = rospy.Rate(10)
        self.j = 0                # for문 돌릴 변수 j 0으로 초기화 

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.mr.odom_callback)           # odom 메세지 토픽 구독자 발행 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1) # cmd_vel  토픽 발행 Twist 메세지 타입
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.mr.scan_callback)          # scan 메세지토픽 구독자 발행 

    # 2차 탈출 메소드 
    def re_escape(self):
        if self.return_start is True:   # 2차 탈출 시작하면
            for i in range(21):
                start_maze = Twist()                      # Twist 메세지 start_maze   
                start_maze.angular.z = -math.radians(90)  # 도착 했으니까 다시 돌아가야함, 머리를 반바퀴 회전 (180도 회전) 다시 미로 방향으로 튼다
                self.cmd_vel_pub.publish(start_maze)      # twist cmd_vel_pub 발행 
                self.rate.sleep()
            self.return_start = False                     # for문 종료후 시작 플래그 false  

        else:                          # 미로 방향으로 머리를 튼 후 
            twist = Twist()
            for i in range(0, Stack.run_count, 1):  # 스택에 쌓았던 카운트 개수 만큼 반복
                self.go_back = True
                
                # 현재 좌표와 스택에 쌍인 좌표 중 0.25 오차만큼의 범위에 들어오면
                if (self.mr.stackX[i] - 0.25 <= self.mr.abs_poseX <= self.mr.stackX[i] + 0.25) and (
                        self.mr.stackY[i] - 0.25 <= self.mr.abs_poseY < self.mr.stackY[i] + 0.25):
                    self.go_back = False                     # 범위 내에 들어왔으니 회전해야함, 직선 주행을 멈추고
                    self.re_rotation = True                  # 회전 하기 위해 플래그 True 설정 
                    if self.re_rotation is True:             # 회전할 경우
                        if self.go_back is False:            # 직선 주행을 멈춘 상태 
                            
                            if self.mr.range_left > self.mr.range_right:       # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 멀 경우
                                for self.j in range(10):
                                    explore = Twist()                          # Twist 메세지 explore 
                                    explore.angular.z = math.radians(90)       # 왼쪽으로 90도 회전하라
                                    self.cmd_vel_pub.publish(explore)          # 회전한 explore로 토픽 발행 
                                    self.rate.sleep()

                                for i in range(5):                             # linear (선속도) 설정 그대로임 
                                    twist.linear.x = 1
                                    self.cmd_vel_pub.publish(twist)
                                    self.rate.sleep()
                                   
                            elif self.mr.range_left < self.mr.range_right:     # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 가까울 경우
                                for self.j in range(10):
                                    explore = Twist()                          # Twist 메세지 explore 
                                    explore.angular.z = -math.radians(90)      # 오른쪽으로 90도 회전하라
                                    self.cmd_vel_pub.publish(explore)          # 회전한 explore로 토픽 발행 
                                    self.rate.sleep()
                                for i in range(5):                             # linear (선속도) 설정 그대로임 
                                    twist.linear.x = 1
                                    self.cmd_vel_pub.publish(twist)
                                    self.rate.sleep()
                        self.go_back = True                                    # 회전 다했으니까 직진해야함 -> 플래그 True 
                        self.re_rotation = False                               # 회전 다했으니까 회전 플래그 false  
                else:                               # 현재 좌표와 스택에 쌓인 좌표 중 오차만큼의 범위에 안들어온 경우 
                    if self.re_rotation is False:
                        if self.go_back is True:        # 오차 없으므로 계속 직진하면 됨. 직진 플래그 true
                            twist.linear.x = 1          # 선속도 설정 
            self.cmd_vel_pub.publish(twist)             # twist cmd_vel_pub 발행 
            self.rate.sleep()
