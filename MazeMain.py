#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Stack(list):  # 좌표저장을 위한 스택
    push = list.append
    run_count = 0

    def check_data(self):
        if not self:
            return True
        else:
            return False

    def check_end(self):
        return self[-1]


class MazeRun:  # 메이즈 1차 탈출 클래스
    stackX = Stack()   # 좌표를 스택에 저장
    stackY = Stack()
    abs_poseX = 0  # 주어진 맵의 좌표
    abs_poseY = 0

    def __init__(self):
        self.driving_forward = True
        self.rotation = False
        self.enter = True
        self.turn = Twist()
        self.twist = Twist()
        self.run_count = 0
        self.rate = rospy.Rate(10)

        self.range_ahead = 1
        self.range_left = 1
        self.range_right = 1

        self.poseX = 0   # 오돔 좌표
        self.poseY = 0

        # 토픽 발행
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    # 스캔 콜백 함수
    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        self.range_left = msg.ranges[int(len(msg.ranges) / 1.3)]  # 1350
        self.range_right = msg.ranges[len(msg.ranges) / 4]  # 450

    # 오돔 좌표와 지도의 좌표를 계산, 오돔 콜백 함수
    def odom_callback(self, msg):
        self.poseX = msg.pose.pose.position.x
        self.poseY = msg.pose.pose.position.y
        self.abs_poseX = self.poseX - 7.92
        self.abs_poseY = self.poseY + 5.30

    # 계산한 좌표 값을 스택에 저장
    def save_pose(self):
        if self.rotation is True:   # 회전을 할 때 마다 저장
            self.stackX.push(self.abs_poseX)
            self.stackY.push(self.abs_poseY)
            self.run_count = self.run_count + 1  # 카운트로 몇 번 쌓이는지 확인
            Stack.run_count = Stack.run_count + 1

    # 미로 탈출 함수
    def maze_escape(self):
        if self.enter is True:  # 탈출 시작 알림
            for i in range(13):
                start_maze = Twist()
                start_maze.angular.z = -math.radians(90)  # 라디안 90도 만큼 돌아라
                self.cmd_vel_pub.publish(start_maze)
                self.rate.sleep()
            self.enter = False  # 처음 시작 시 90도 회전 후 시작 플래그 False
        else:
            if self.rotation is True:  # 회전할 경우
                for i in range(10):
                    self.cmd_vel_pub.publish(self.turn)
                    self.rate.sleep()
                self.rotation = False
            else:
                if self.driving_forward is True:  # 주행 중인 상태
                    if self.range_ahead < 0.7:    # 정면 물체 인식 거리가 0.7 보다 작으면
                        self.driving_forward = False  # 직선 주행 정지
                else:
                    if self.range_ahead > 0.7:  # 정면 물체 인식 거리가 0.7 보다 크면
                        self.driving_forward = True  # 직선 주행 계속
                twist = Twist()
                if self.driving_forward is True:  # 직선 주행 중일 경우
                    twist.linear.x = 1
                    if self.range_left > self.range_right:  # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 멀 경우
                        self.turn.angular.z = math.radians(90)  # 왼쪽으로 90도 회전하라
                    elif self.range_left < self.range_right:  # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 가까울 경우
                        self.turn.angular.z = -math.radians(90)  # 오른쪽으로 90도 회전하라
                else:  # 직선 주행이 아닐경우 회전
                    self.rotation = True
                self.cmd_vel_pub.publish(twist)
                self.rate.sleep()


# 1차 탈출, 2차 탈출 시작 조건을 위한 클래스
class Begin:
    def __init__(self):
        self.mr = MazeRun()  # 1차 탈출 클래스 객체 선언
        self.tb = TakeBack()  # 2차 탈출 클래스 객체 선언
        self.target = False  # 도착 알림 플래그

    # 출발, 도착 좌표 지점 인식 함수
    # 출발지 좌표 = (-7.92, 5.3)
    # 목적지 좌표 = (7.95, -5.15)
    def get_target(self):
        while not rospy.is_shutdown():
            # 맵의 목적지 좌표 일정 범위에 들어가면 target = False 바꾸고 멈춰라
            if (-8.55 < self.mr.abs_poseX < -7.55) and (
                    5.05 < self.mr.abs_poseY < 5.55) and self.target is True:  # End Range
                break

            # 맵의 출발지 좌표 일정 범위에 들어가면 target = True 바꾸고
            if ((7.75 < self.mr.abs_poseX < 8.15) and (
                    -5.35 < self.mr.abs_poseY < -4.95)) or self.target is True:  # Goal Range
                self.target = True
                self.tb.re_escape()   # 2차 탈출 시작
            else:
                self.mr.maze_escape()  # 아닐 경우 1차 탈출
                self.mr.save_pose()  # 좌표 값 저장


class TakeBack:
    def __init__(self):
        self.mr = MazeRun()

        self.return_start = True
        self.go_back = True
        self.re_rotation = False
        self.rate = rospy.Rate(10)
        self.j = 0

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.mr.odom_callback)  # odom
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.mr.scan_callback)

    # 2차 탈출 함수
    def re_escape(self):
        if self.return_start is True:   # 2차 탈출 시작하면
            for i in range(21):
                start_maze = Twist()
                start_maze.angular.z = -math.radians(90)  # 180도 회전, 다시 미로 방향으로 튼다
                self.cmd_vel_pub.publish(start_maze)
                self.rate.sleep()
            self.return_start = False

        else:
            twist = Twist()
            for i in range(0, Stack.run_count, 1):  # 스택에 쌓았던 카운트 개수 만큼 반복
                self.go_back = True
                # 현재 좌표와 스택에 쌍인 좌표 중 0.25 오차만큼의 범위에 들어오면
                if (self.mr.stackX[i] - 0.25 <= self.mr.abs_poseX <= self.mr.stackX[i] + 0.25) and (
                        self.mr.stackY[i] - 0.25 <= self.mr.abs_poseY < self.mr.stackY[i] + 0.25):
                    self.go_back = False  # 직선 주행을 멈추고
                    self.re_rotation = True  # 회전
                    if self.re_rotation is True:  # 회전할 경우
                        if self.go_back is False:
                            # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 멀 경우
                            if self.mr.range_left > self.mr.range_right:
                                for self.j in range(10):
                                    explore = Twist()
                                    explore.angular.z = math.radians(90)  # 왼쪽으로 90도 회전하라
                                    self.cmd_vel_pub.publish(explore)
                                    self.rate.sleep()

                                for i in range(5):
                                    twist.linear.x = 1
                                    self.cmd_vel_pub.publish(twist)
                                    self.rate.sleep()
                                    # 왼쪽 벽과의 거리가 오른쪽 벽과의 거리보다 가까울 경우
                            elif self.mr.range_left < self.mr.range_right:
                                for self.j in range(10):
                                    explore = Twist()
                                    explore.angular.z = -math.radians(90)  # 오른쪽으로 90도 회전하라
                                    self.cmd_vel_pub.publish(explore)
                                    self.rate.sleep()
                                for i in range(5):
                                    twist.linear.x = 1
                                    self.cmd_vel_pub.publish(twist)
                                    self.rate.sleep()
                        self.go_back = True
                        self.re_rotation = False
                else:
                    if self.re_rotation is False:
                        if self.go_back is True:
                            twist.linear.x = 1
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
