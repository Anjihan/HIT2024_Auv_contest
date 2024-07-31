#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
import os
from time import sleep, time

from std_msgs.msg import Int32, String, Float32, Float64
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

import tf
import tf_conversions
from geometry_msgs.msg import Quaternion

from cv_bridge import CvBridge
import cv2
import numpy as np

from qr_test import qr_code_reader

from sensor_msgs.msg import Joy
import pyrealsense2 as rs

ref_zero = 165 #IMU데이터 기반으로 수정 필요  /Y가 전방
D2R = 3.141592/180
R2D = 180/3.141592



class MainLoop:
    
    def __init__(self):
        self.qr_reader = qr_code_reader()
        self.bridge = CvBridge()

        #-----------------------------------------------------<<-mission 상태 변수->>--------------------------------------------------#
        self.mission_C = False
        self.mission_D_L = False
        self.mission_D_R = False
        self.mission_E = False
        self.mission_F = False
     
        #------------------------------------------------mission_C_녹색골대-------------------------------------------------------------#
        #QR미션 -> QR인식 후 동작 작동여부 결정 flag
        self.QR_L_right = False
        self.QR_R_right = False
        self.stop_t1 = 0.0
        self.static_flag_C = False         # depth홀드 후 하강을 시작하는 시간을 구하기 위한 lock

        #-----------------------------------------------------<<리얼센스 사용 미션인 경우>>--------------------------------------------------#
        #
        #
        #-----------------------------------------------------<<리얼센스 사용 미션인 경우>>--------------------------------------------------#

        self.obstacle_img = []
        self.originalImg = []

        # publisher : mainAlgorithm에서 계산한 제어 전달
        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        
        self.AUV_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # AUV speed 전진,후진,레터럴,상승,하강 -> 각각 속도에 맞추어 튜닝해서 정리
        self.AUV_mode_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1) # AUV mode -> stabilize, depth hold 모드 제어
        self.AUV_angle_pub = rospy.Publisher("/commands/servo/orientation", Float64, queue_size=1) # AUV angle -> yaw 속도 제어 or Imu 데이터 기반 자세제어 -> C, D 미션에서는 안쓸예정, 추후 필요한 미션 담당자가 사용할 것

        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1) # 카메라 이미지 ROS를 통해서 publish


        #제어 입력으로 넣을 조이스틱 데이터 예시 -> 수정해야될 수 있음
        # joy_msg = Joy()
        # joy_msg.axes = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming the left joystick forward is axes[1] = 0.2
        # joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # All buttons are not pressed


        # subscriber : joy데이터를 통해서 미션 시작, qr인식 확인, 자석 센서값 확인, 음향 등대값 확인
        rospy.Subscriber('/joy', Joy, self.joystickCallback)
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback)

        rospy.Subscriber("usb_cam/image_rect_color", Image, self.QR_Callback_L)
        rospy.Subscriber("usb_cam/image_rect_color", Image, self.QR_Callback_R)        
        rospy.Subscriber("sign_id", Int32, self.child_sign_callback)



        rospy.Subscriber("자석 센서값 쓰게된다면", Float32, self.NS_callback)
        rospy.Subscriber("음향 등대값 쓰게된다면", String,  self.Sound_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
   

    def image_callback(self, data): #ROS 이미지 받아와서 CV에서 처리가능하게 바꿈 -> (추후 영상처리 필요하면 이 함수에서 분기하세요) -> 처리한 CV영상 다시 ROS형식으로 publish
        # ROS 이미지 메시지에서 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # 원본 이미지를 ROS 메시지로 변환
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # ROS 메시지 퍼블리시
        self.image_pub.publish(ros_image)
   
    def quaternion_to_yaw(self, quat): #IMU 데이터 처리함수 by 승훈 - 오키나와 해양수중로봇대회
        # 쿼터니안을 오일러 각도로 변환
        euler = tf_conversions.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = euler[2]
        # yaw = 195*D2R

        # Yaw 각도를 조정하여 X축이 위를 향할 때 220도가 되도록 함
        # 220도를 라디안으로 변환
        target_yaw_rad = ref_zero * D2R #radian

        # Yaw 값을 조정
        adjusted_yaw = 2*3.141592 -yaw - target_yaw_rad + 60*D2R#-165*D2R

        print('before_yaw : ',yaw*R2D)
        # Yaw 값이 2π 이상이면 2π를 빼서 범위를 -π ~ π로 조정
        adjusted_yaw = adjusted_yaw % (2 * 3.141592)
        return adjusted_yaw
    
    def imu_callback(self, data): #IMU 데이터 처리함수 by 승훈 - 오키나와 해양수중로봇대회
        # IMU 데이터 콜백
        yaw = self.quaternion_to_yaw(data.orientation)
        print('after_yaw',(yaw)*R2D)
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         tf_conversions.transformations.quaternion_from_euler(0, 0, yaw),
                         rospy.Time.now(),
                         "robot",
                         "sonar_frame")

    def NS_callback(self, _data): #자석 어떻게 처리할지 몰라서 일단 만들어둠
        pass
    
    def Sound_callback(self, _data): #음향등대 어떻게 처리할지 몰라서 일단 만들어둠
        pass

    def timerCallback(self, _event): #mainAlgorithm 30Hz 주기로 돌리는함수
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

#===========================================================================#
#===========================================================================#
#=============================== 미션함수코딩구역 ==============================#
#===========================================================================#
#===========================================================================#
    def QR_code_Drive_L(self, _data): #mission D QR 코드 미션
        pass

    def QR_code_Drive_R(self, _data): #mission D QR 코드 미션
        pass

    
    def green_Goal_Drive(self): # mission C 녹색 골대 지나는 미션
        rospy.loginfo("MISSION: GOAL")
        t2 = rospy.get_time() #-> 갱신되면서 변할 시간 (변수) 
         
        # depth hold 0.5m 이동 후 천천히 전진 (5)초 -> 괄호 친 숫자는 튜닝해야되는 변수

        # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
        if self.static_flag_C == False:
            self.stop_t1 = rospy.get_time() # start time -> 미션 시작하는 시간(상수)        
            self.mode_msg.data = 1 #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold
            self.static_flag_C = True
            
        # (2)초간 depth hold에서 아래로 -> 0.5m 맞추기
        elif t2 - self.stop_t1 <= 2:
            self.speed_msg.data = 0 #하강 joystick 입력으로 바꾸면됨 ========================================================================= 하강
            
        # (5)초간 이전 각도로 주행 -> 골대 지나서 얼만큼 가는지 튜닝해야됨
        elif t2 - self.stop_t1 <= 7: #시간은 누적으로 계산하세요            
            self.speed_msg.data = 1000 #전진 joystick 입력으로 바꾸면됨 ========================================================================= 전진
  
        else:
            self.mission_C = False
                            
        self.AUV_speed_pub.publish(self.speed_msg)
        self.AUV_mode_pub.publish(self.mode_msg)

    def defaultDrive(self): #기본 AUV 알고리즘에서 주행은 정지로 설정 -> 만약 아무런 커맨드를 안받았을 때는 정지해있도록
        rospy.loginfo("MISSION: Default Driving")
        self.speed_msg.data = 0 #정지 중립위치 joystick 입력으로 바꾸면됨 ========================================================================= 정지
        self.mode_msg.data = 1 #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold

        self.AUV_speed_pub.publish(self.speed_msg)
        self.AUV_mode_pub.publish(self.mode_msg)
#========================================================================================#
#========================================================================================#
#=========================== 조이스틱으로 미션 키기 위한 지정 함수들  ===========================#
#========================================================================================#
#========================================================================================#       
    def joystickCallback(self, data): #조이스틱 데이터 처리하는 callback 함수
        # Update joystick axes and buttons from the received message
        self.joystick_axes = data.axes
        self.joystick_buttons = data.buttons

        # Optional: Print joystick data for debugging
        rospy.loginfo("Axes: %s", self.joystick_axes)
        rospy.loginfo("Buttons: %s", self.joystick_buttons)

        # 조이스틱 버튼으로 미션 활성화 (예: 세모 버튼이 버튼 인덱스 1이라고 가정)
        if data.buttons[1] == 1:  # 세모 버튼이 눌렸을 때 -> 일단 미션 C 가 이런거라고 가정했습니다 수정 요망 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 미션C 조이스틱 버튼
            self.mission_C = True
            self.mission_D_L = False
            self.mission_D_R = False        
            self.mission_E = False
            self.mission_F = False
    
        elif data.buttons[1] == 2:
            self.mission_C = False
            self.mission_D_L = True
            self.mission_D_R = False        
            self.mission_E = False
            self.mission_F = False

        elif data.buttons[1] == 3:
            self.mission_C = False
            self.mission_D_L = False
            self.mission_D_R = True        
            self.mission_E = False
            self.mission_F = False

        elif data.buttons[1] == 4:
            self.mission_C = False
            self.mission_D_L = False
            self.mission_D_R = False               
            self.mission_E = True
            self.mission_F = False  
        else:
            self.mission_C = False
            self.mission_D_L = False
            self.mission_D_R = False              
            self.mission_E = False
            self.mission_F = True

        # 필요에 따라 다른 버튼 처리 추가
        
        
    def mainAlgorithm(self):
        rospy.loginfo("MAIN")
        # C. 녹색 골대 지나는 미션 
        if self.mission_C == True:
            self.green_Goal_Drive()
        
        # D. QR 코드 읽는 미션 L
        elif self.mission_D == True:
            self.QR_code_Drive_L()

        # D. QR 코드 읽는 미션 R
        elif self.mission_D == True:
            self.QR_code_Drive_R()

        # # E. 음향등대
        elif self.mission_E == True:
            self.defaultDrive()
            
        # # F. 꼬챙이니까 이건 돌아오는거 용으로 일단 둠
        elif self.mission_F == True:
            self.defaultDrive()
            
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