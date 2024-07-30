#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import os
from time import sleep, time

from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np

from qr_test import qr_code_reader
from evaluateColor import Evaluator #폴더에서 사용할 class 정하면됨

from sensor_msgs.msg import Joy
import pyrealsense2 as rs



class MainLoop:
    
    def __init__(self):
        self.qr_reader = qr_code_reader()
        self.bridge = CvBridge()
        self.evalutator = Evaluator()

        #-----------------------------------------------------<<-mission 상태 변수->>--------------------------------------------------#
        self.mission_C = False
        self.mission_D = False
        self.mission_E = False
        self.mission_F = False
     
        #-----------------------------------------------------<<-mission 상태 변수->>--------------------------------------------------#
        # for static obstacle => mission C 골대 미션
        self.is_doing_static_mission = False
        self.static_t1 = 0.0           # 부표 미션 시작 시간
        self.static_flag = False       # 부표 미션 시간을 구하기 위한 lock




        #QR미션 -> QR인식 후 동작 작동여부 결정 flag
        self.QR_L_right = False
        self.QR_R_right = False

        #------------------------------------------------mission_C_녹색골대-------------------------------------------------------------#
        self.is_goal_start = False # mission C 시작 여부 판단
        self.stop_t1 = 0.0
        self.down_flag = False         # depth홀드 후 하강을 시작하는 시간을 구하기 위한 lock
        self.angle_child_before = 0.57


        #-----------------------------------------------------<<리얼센스 사용 미션인 경우>>--------------------------------------------------#
        #
        #
        #-----------------------------------------------------<<리얼센스 사용 미션인 경우>>--------------------------------------------------#

        self.obstacle_img = []
        self.originalImg = []

        # publisher : mainAlgorithm에서 계산한 속력과 조향각을 전달함
        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        self.webot_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # motor speed
        self.webot_angle_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1) # servo angle
        self.webot_lane_pub = rospy.Publisher("/currentLane", Float64, queue_size=1) # 1 = left, 2 = right

        self.AUV_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # AUV speed 전진,후진,레터럴,상승,하강 -> 각각 속도에 맞추어 튜닝해서 정리
        self.AUV_mode_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1) # AUV mode -> stabilize, depth hold 모드 제어
        self.AUV_angle_pub = rospy.Publisher("/commands/servo/orientation", Float64, queue_size=1) # AUV angle -> yaw 속도 제어 or Imu 데이터 기반 자세제어 -> C, D 미션에서는 안쓸예정, 추후 필요한 미션 담당자가 사용할 것

        #제어 입력으로 넣을 조이스틱 데이터 예시 -> 수정해야될 수 있음
        # joy_msg = Joy()
        # joy_msg.axes = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming the left joystick forward is axes[1] = 0.2
        # joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # All buttons are not pressed


        # subscriber : joy데이터를 통해서 미션 시작, qr인식 확인, 자석 센서값 확인, 음향 등대값 확인
        rospy.Subscriber('/joy', Joy, self.joystickCallback)

        rospy.Subscriber("usb_cam/image_rect_color", Image, self.QR_Callback_L)
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.QR_Callback_R)        

        rospy.Subscriber("자석 센서값 쓰게된다면", Float32, self.NS_callback)
        rospy.Subscriber("음향 등대값 쓰게된다면", String,  self.Sound_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)

   
   
    def joystickCallback(self, data):
        # Update joystick axes and buttons from the received message
        self.joystick_axes = data.axes
        self.joystick_buttons = data.buttons
        
        # Optional: Print joystick data for debugging
        rospy.loginfo("Axes: %s", self.joystick_axes)
        rospy.loginfo("Buttons: %s", self.joystick_buttons)

    def NS_callback(self, _data):
        pass
    
    def Sound_callback(self, _data):
        pass

    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
            pass
        except:
            pass
    
    def QR_Callback_L(self, _data): # QR_detect.py에서 넘겨준 qr값이 맞는지 확인하는 함수
        # detect lane
        
        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.originalImg = cv2_image.copy()
        
        if self.qr_reader.qr_return(self.originalImg) == 1:
            self.QR_L_right = True
        else:
            self.QR_R_right = False
        
    def QR_Callback_R(self, _data): # QR_detect.py에서 넘겨준 qr값이 맞는지 확인하는 함수
        # detect lane
        
        cv2_image = self.bridge.imgmsg_to_cv2(_data)
        self.originalImg = cv2_image.copy()
        
        if self.qr_reader.qr_return(self.originalImg) == 2:
            self.QR_R_right = True
        else:
            self.QR_L_right = False
    

    def green_Goal_Drive(self): # mission C 녹색 골대 지나는 미션
        rospy.loginfo("MISSION: GOAL")
        t2 = rospy.get_time()
         
        # depth hold 0.5m 이동 후 천천히 전진 (5)초 -> 괄호 친 숫자는 튜닝해야되는 변수

        # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
        if self.down_flag == False:
            self.stop_t1 = rospy.get_time() # start time
            self.is_goal_start = True
           
            self.mode_msg.data = 1 #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold
            
            self.down_flag = True
            
        # (2)초간 depth hold에서 아래로 -> 0.5m 맞추기
        elif t2 - self.stop_t1 <= 2:
            self.speed_msg.data = 0 #하강 joystick 입력으로 바꾸면됨 ========================================================================= 하강
            
        # (5)초간 이전 각도로 주행 -> 골대 지나서 얼만큼 가는지 튜닝해야됨
        elif t2 - self.stop_t1 <= 7: #시간은 누적으로 계산하세요            
            self.speed_msg.data = 1000 #전진 joystick 입력으로 바꾸면됨 ========================================================================= 전진
        else:
            pass
                            
        self.AUV_speed_pub.publish(self.speed_msg)
        self.AUV_mode_pub.publish(self.mode_msg)

    def defaultDrive(self): #기본 AUV 알고리즘에서 주행은 정지로 설정 -> 만약 아무런 커맨드를 안받았을 때는 정지해있도록
        rospy.loginfo("MISSION: Default Driving")
        self.speed_msg.data = 0 #정지 중립위치 joystick 입력으로 바꾸면됨 ========================================================================= 정지
        self.mode_msg.data = 1 #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold

        self.AUV_speed_pub.publish(self.speed_msg)
        self.AUV_mode_pub.publish(self.mode_msg)
        
        
        
    def mainAlgorithm(self):
        rospy.loginfo("MAIN")
        # C. 녹색 골대 지나는 미션 
        if self.initDriveFlag == True:
            self.green_Goal_Drive()
        
        # D. QR 코드 읽는 미션
        elif self.is_safe == False and self.isDynamicMission == True:
            self.dynamicObstacle()

        # # E. child protect driving
        # elif self.is_child_detected == True:
        #     self.childProtectDrive()

        # # F. rubbercone mission
        # elif self.is_rubbercone_mission == True:
        #     self.rubberconeDrive()
        #     self.publish()
        
        # 기본 상태 -> 정지
        else:
            self.defaultDrive()      


def nothing(x):
    pass

def run():

    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()