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

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

from sensor_msgs.msg import Joy
import pyrealsense2 as rs


ref_zero = 165 #IMU데이터 기반으로 수정 필요  /Y가 전방
D2R = 3.141592/180
R2D = 180/3.141592

class QRdetect:
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX


    def read_frame(self, frame):
        # QR 코드 정보 디코딩
        barcodes = pyzbar.decode(frame)
        barcode_data = []
        for barcode in barcodes:
            barcode_info = barcode.data.decode('utf-8')
            barcode_data.append(barcode_info)
        return barcode_data


class MainLoop:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.qr_detector = QRdetect()  # QRdetect 클래스 인스턴스 생성
        
        #-----------------------------------------------------<<-mission 상태 변수->>--------------------------------------------------#
        #미션 상태 flag입니다. 이거 켜지면 해당 미션에 계속 들어감 -> 미션이 완전히 끝났으면 꺼줘야됨
        #추후 조이스틱 버튼을 더 설정하면 미션을 강제로 끄도록 코드 추가도 가능합니다
        self.mission_C = False
        self.mission_D_L = False
        self.mission_D_R = False
        self.mission_E = False
        self.mission_F = False
     
        #------------------------------------------------mission_C_녹색골대-------------------------------------------------------------#
        #QR미션 -> QR인식 후 동작 작동여부 결정 flag
        self.QR_L_right = False
        self.QR_R_right = False
        self.stop_t1_C = 0.0
        self.static_flag_C = False         # depth홀드 후 하강을 시작하는 시간을 구하기 위한 lock
        #------------------------------------------------mission_D_L QR-------------------------------------------------------------#
        self.mission_D_L_bottom_start = False #바닥 qr로 가기 위한 flag
        self.mission_D_L_front_aline = False # 수면 QR을 화면 가운데로 정렬시키기 위한 함수를 시작시키기 위한 flag
        self.static_flag_D_L = False         # depth홀드 후 왼쪽 전진 시작하는 시간을 구하기 위한 lock        
        self.static_flag_D_L_2 = False
        self.stop_t1_D_L = 0.0
        self.stop_t1_D_L_2 = 0.0
        self.mission_D_L_front_start = False #정면QR 박치기 하기위해 QR인식가능지점까지 가는 flag
        self.mission_D_L_front_end = False #정면QR 박치기 하는 동작을 위한 flag -> 마지막 높은 속도로 전진                 
        #------------------------------------------------mission_D_R QR-------------------------------------------------------------#


        self.stop_t1_D_R = 0.0
        self.static_flag_D_R = False         # depth홀드 후 우측 전진 시작하는 시간을 구하기 위한 lock        
        #-----------------------------------------------------<<HSV값 사용 미션인 경우>>--------------------------------------------------#
        #미리 리허설에서 경기장의 hsv값을 찾아서 설정해둔 다음에 상수로 넣어놓고 Want_color 메소드를 이용하면 원하는 색상으로 마스킹할 수 있습니다
        self.lower_hsv1 = (0, 0, 0)         # HSV range for the first camera 
        self.upper_hsv1 = (180, 255, 255)
        self.lower_hsv2 = (20, 100, 100)    # HSV range for the second camera (example values)
        self.upper_hsv2 = (30, 255, 255)        
        #-----------------------------------------------------<<카메라 사용 미션인 경우>>--------------------------------------------------#

        self.obstacle_img = []
        self.originalImg = []

        # publisher : mainAlgorithm에서 계산한 제어 전달
        rospy.Timer(rospy.Duration(1.0/30.0), self.timerCallback)
        
        # self.AUV_speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1) # AUV speed 전진,후진,레터럴,상승,하강 -> 각각 속도에 맞추어 튜닝해서 정리
        # self.AUV_mode_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1) # AUV mode -> stabilize, depth hold 모드 제어
        # self.AUV_angle_pub = rospy.Publisher("/commands/servo/orientation", Float64, queue_size=1) # AUV angle -> yaw 속도 제어 or Imu 데이터 기반 자세제어 -> C, D 미션에서는 안쓸예정, 추후 필요한 미션 담당자가 사용할 것
        # 위 토픽 JOY로 대체
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8  # 예시: 8개의 축
        self.joy_msg.buttons = [0] * 12  # 예시: 12개의 버튼        
        
        self.AUV_pub = rospy.Publisher("/joy", Joy, queue_size=1)
            
        self.image_pub1 = rospy.Publisher("/camera1/image_raw", Image, queue_size=10)
        self.image_pub2 = rospy.Publisher("/camera2/image_raw", Image, queue_size=10)

        self.cap1 = cv2.VideoCapture(4) # 전면카메라 -> 내 노트북에서는 VideoCapture(6)임 다르면 추후 수정 요망
        self.cap2 = cv2.VideoCapture(6) # 위 카메라  -> 추후 번호 찾아서 맞추세요 

        # 카메라 두개 다 제대로 켜졌는지 확인
        if not self.cap1.isOpened():
            rospy.logerr("Failed to open camera 6_foward")
        if not self.cap2.isOpened():
            rospy.logerr("Failed to open camera 0_up")

        #제어 입력으로 넣을 조이스틱 데이터 예시 -> 수정해야될 수 있음
        # joy_msg = Joy()
        # joy_msg.axes = [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming the left joystick forward is axes[1] = 0.2
        # joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # All buttons are not pressed


        # subscriber : joy데이터를 통해서 미션 시작, qr인식 확인, 자석 센서값 확인, 음향 등대값 확인
        rospy.Subscriber('/joy', Joy, self.joystickCallback) #조이스틱 값으로 어떤 미션 실행시킬지 데이터 받는 Subscriber입니다
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback)

        # rospy.Subscriber("자석 센서값 쓰게된다면", Float32, self.NS_callback)
        # rospy.Subscriber("음향 등대값 쓰게된다면", String,  self.Sound_callback) # lidar 에서 받아온 object 탐지 subscribe (warning / safe)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    def camera_get(self): #카메라 두개의 이미지를 동시에 받아오는 함수
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()
        
        if ret1:
            img_msg1 = self.bridge.cv2_to_imgmsg(frame1, "bgr8")
            self.image_pub1.publish(img_msg1)
        else:
            rospy.logwarn("Failed to capture frame from camera 0.")
        
        if ret2:
            img_msg2 = self.bridge.cv2_to_imgmsg(frame2, "bgr8")
            self.image_pub2.publish(img_msg2)
        else:
            rospy.logwarn("Failed to capture frame from camera 6.")
            
        return ret1, frame1, ret2, frame2

    def image_callback(self, data): #ROS 이미지 받아와서 CV에서 처리가능하게 바꿈 -> (추후 영상처리 필요하면 이 함수에서 분기하세요) -> 처리한 CV영상 다시 ROS형식으로 publish
        # ROS 이미지 메시지에서 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # 원본 이미지를 ROS 메시지로 변환
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # ROS 메시지 퍼블리시
        self.image_pub.publish(ros_image)
    #=========  QR 인식 : 리스트 형식으로 출력이 되어서 숫자로 처리가능하게 바꿨음========#
    def QR_read_front(self):
        ret1, frame1, _, _ = self.camera_get()
        if ret1:
            barcode_data1 = self.qr_detector.read_frame(frame1)
            rospy.loginfo(f"front Camera QR 코드 데이터: {barcode_data1}")
            if barcode_data1:
                try:
                    return int(barcode_data1[0])
                except ValueError:
                    rospy.logwarn(f"Unable to convert QR code data '{barcode_data1[0]}' to integer.")
        return 0

    def QR_read_top(self):
        _, _, ret2, frame2 = self.camera_get()
        if ret2:
            barcode_data2 = self.qr_detector.read_frame(frame2)
            rospy.loginfo(f"Top Camera QR 코드 데이터: {barcode_data2}")
            if barcode_data2:
                try:
                    return int(barcode_data2[0])
                except ValueError:
                    rospy.logwarn(f"Unable to convert QR code data '{barcode_data2[0]}' to integer.")
        return 0
    #=====================================================================================#
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
    
#===========================================================================#
#===========================================================================#
#=============================== 미션함수코딩구역 ==============================#
#===========================================================================#
#===========================================================================#
    #QR미션은 가정이 많음 -> L 미션인 경우 왼쪽 -> 전진으로 화면 중앙에 QR을 위치시키고 다가가는 중이라고 가정했을 때
    #1. 수면 QR이 읽히는 지점이 이미 바닥 QR은 지나간 경우
    #2. 수면 QR이 읽히는 지점보다 앞에 바닥 QR이 존재할 경우
    #3. 바닥 QR의 깊이 정보, 바닥 QR과 수면 QR의 직선거리, 자석의 세기 모두 미정
    
    #==> 2024.08.03기준 코드는 가장 보편적인 상황을 가정하고 짰음 -> 추후 수정 필요
    #프로세스 설명 
    #1. 수면 QR을 먼저 찾는다 -> 5m정도 앞에 있다는 코멘트가 룰 설명지에 있으므로 2.5m정도 앞으로 간다
    #2. 0.3m^2 크기의 수면 QR이 영상처리를 하기에 적절한 거리만큼 존재한다고 가정 (일단 2.5m로 가정) 이때 바닥 QR을 아직 지나치지 않았다는 가정
    #3. 색상의 가운데 점을 리얼센스 화면의 가운데 구간으로 이동시켜서 ROV와 QR황색판 정렬
    #4. 바닥으로 depth hold 깊이를 4.5m ~ 6.5m로 맞추기 위한 하강(바닥 자석을 읽기 위한 setup)
    #5. 위치 정렬은 3번에서 했으므로 top camera가 QR정보를 읽을때까지 천천히 전진 and QR정보가 들어오면 stop -> 만약 자석 이래도 못읽으면 추가로 더 내려가던지 동작 필요
    #6. 다시 depth hold 깊이를 0.5m로 상승(수면 QR에 박치기하기 위한 setup)
    #7. 정면 camera QR값이 들어올때까지 전진 and stop
    #8. 빠른속도로 전진 and 미션 flag(mission_D) False로 반환 후 종료 
    def QR_code_Drive_L(self, _data): #mission D QR 코드 미션 L구간
        rospy.loginfo("MISSION: QR_LL")
        self.qr_front_result = self.QR_read_front()
        self.qr_top_result = self.QR_read_top()
        
        t2 = rospy.get_time() #-> 갱신되면서 변할 시간 (변수) 

        # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
        if self.static_flag_D_L == False:
            self.stop_t1_D_L = rospy.get_time() # start time -> 미션 시작하는 시간(상수)        
            # self.mode_msg.data = 1 #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold
            self.joy_msg.buttons[3] = 1 #lsh  
            # self.joy_msg.axis
            self.static_flag_D_L = True
  
        # (2)초간 depth hold 상태에서 왼쪽으로 -> 화각에 왼쪽 라인의 QR만 보이게하기 위함 -> 왼쪽 라인이 카메라 중심에 오도록 맞춰야됨 -------튜닝 필요
        elif t2 - self.stop_t1_D_L <= 2:
            # self.speed_msg.data = 0 #왼쪽 레터럴 joystick 입력으로 바꾸면됨 ========================================================================= 왼쪽 레터럴
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)
            self.joy_msg.axes[3] = 0.5 #lsh 
            
            
        # (5)초간 이전 각도로 주행 -> 전진해서 수면 QR에 가까워지기 위함 -> 2.5m 정도 왔다고 가정 ---------------튜닝 필요
        elif t2 - self.stop_t1_D_L <= 7: #시간은 누적으로 계산하세요            
            # self.speed_msg.data = 1000 #전진 joystick 입력으로 바꾸면됨 ========================================================================= 전진
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)
            self.joy_msg.axes[4] = 0.5 #lsh       
  
        elif t2 - self.stop_t1_D_L <= 8: # else로 빼면 계속 켜질꺼같아서 일단 8초로 해둠
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)
            self.mission_D_L_front_aline = True #정면카메라 색상 정렬 코드 flag ON
             
        else:
            pass
       
        if self.mission_D_L_front_aline == True: #정면카메라 색상 정렬 부분
            rospy.loginfo("MISSION: QR_LL_aline")            
            while not rospy.is_shutdown():
                ret1, frame1, ret2, frame2 = self.camera_get()

                if ret1:
                    number1 = self.center_mission(frame1, self.lower_hsv1, self.upper_hsv1, "Camera 1")
                    rospy.loginfo(f"Camera 1 detected number: {number1}")

                if number1 == 1:
                    # self.speed_msg.data = 0 #우측 레터럴 joystick 입력으로 바꾸면됨 ========================================================================= 우측 레터럴
                    self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                    self.joy_msg.axes[3] = -0.5 #lsh
                      
                elif number1 == 2:
                    # self.speed_msg.data = 0 #왼쪽 레터럴 joystick 입력으로 바꾸면됨 ========================================================================= 왼쪽 레터럴
                    self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                    self.joy_msg.axes[3] = 0.5 #lsh  
                elif number1 == 3:
                    # self.speed_msg.data = 0 #하강 joystick 입력으로 바꾸면됨 ========================================================================= 하강
                    self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                    self.joy_msg.axes[1] = 0.5 #lsh  
                elif number1 == 4:
                    # self.speed_msg.data = 0 #상승 joystick 입력으로 바꾸면됨 ========================================================================= 상승
                    self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                    self.joy_msg.axes[1] = -0.5 #lsh  
                else:
                    # self.speed_msg.data = 0 #정지 중립위치 joystick 입력으로 바꾸면됨 ========================================================================= 정지
                    self.defaultDrive() 
                    self.mission_D_L_bottom_start = True  #바닥 qr로 가기 위한 flag            
                    self.mission_D_L_front_aline = False
                    break
                
        if self.mission_D_L_bottom_start == True: #내려간다음 top camera가 qr을 인식할때까지 전진 -> 자석 읽었다고 생각
            rospy.loginfo("MISSION: QR_LL_bottom")        
            t3 = rospy.get_time() #-> 갱신되면서 변할 시간 (변수)
            # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
            if self.static_flag_D_L_2 == False:
                self.stop_t1_D_L_2 = rospy.get_time() # start time -> 미션 시작하는 시간(상수)        
                # self.mode_msg.data = 1 #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold
                self.joy_msg.buttons[3] = 1 #lsh  
                self.static_flag_D_L_2 = True

            # (2)초간 depth hold 상태에서 아래로 -> 바닥 QR을 보기 위해서 최대한 내려야됨 ------------튜닝 필요
            elif t3 - self.stop_t1_D_L_2 <= 2:
                # self.speed_msg.data = 0 #왼쪽 레터럴 joystick 입력으로 바꾸면됨 ========================================================================= 왼쪽 레터럴
                self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                self.joy_msg.axes[3] = 0.5 #lsh  
            # 바닥QR을 찾기위해서 천천히 전진          
            else:
                # self.speed_msg.data = 0 #전진 joystick 입력으로 바꾸면됨 ========================================================================= 전진
                self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                self.joy_msg.axes[4] = 0.5 #lsh              
                if (self.qr_top_result == 1):
                    # self.speed_msg.data = 0 #정지 중립위치 joystick 입력으로 바꾸면됨 ========================================================================= 정지
                    self.defaultDrive()
                    self.mission_D_L_front_start = True
                    self.mission_D_L_bottom_start = False
                    
        if self.mission_D_L_front_start == True:
            rospy.loginfo("MISSION: QR_LL_front")                    
            # self.speed_msg.data = 0 #전진 joystick 입력으로 바꾸면됨 ========================================================================= 전진 
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)
            self.joy_msg.axes[4] = 0.5 #lsh                         
            if (self.qr_front_result == 1):
                # self.speed_msg.data = 0 #정지 중립위치 joystick 입력으로 바꾸면됨 ========================================================================= 정지
                self.joy_msg.axes = [0] * len(self.joy_msg.axes)
                self.joy_msg.axes[4] = 0 #lsh  
                self.mission_D_L_front_end = True
                self.mission_D_L_front_start = False            
        
        if self.mission_D_L_front_end == True:
            rospy.loginfo("MISSION: QR_LL_front_end")            
            # self.speed_msg.data = 0 #빠르게 전진 joystick 입력으로 바꾸면됨 ========================================================================= 빠르게 전진  
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)
            self.joy_msg.axes[4] = 0.7 #lsh                     
            self.mission_D_L = False    
                                        
        # self.AUV_speed_pub.publish(self.speed_msg)
        # self.AUV_mode_pub.publish(self.mode_msg)
        self.AUV_pub.publish(self.joy_msg) #lsh

#=================================================================
    def QR_code_Drive_R(self, _data): #mission D QR 코드 미션 R구간 -> L에 대하여 반대로 코드 짜면됨
        rospy.loginfo("MISSION: QR_RR")
        t2 = rospy.get_time() #-> 갱신되면서 변할 시간 (변수) 

        # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
        if self.static_flag_D_R == False:
            self.stop_t1 = rospy.get_time() # start time -> 미션 시작하는 시간(상수)        
            self.joy_msg.buttons[3] = 1 #lsh   #depth hold joystick 입력으로 바꾸면됨 =========================================================================depth hold
            self.static_flag_D_R = True

    def center_point(self, image): #지정한 영역 컨투어 중심에 빨간점을 찍는 메소드
        if image is None or image.size == 0:
            rospy.logwarn("Received an empty image in center_point method.")
            return None

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply binary thresholding
        _, binary = cv2.threshold(blurred, 1, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            rospy.loginfo("No contours found.")
            return None

        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        
        if M["m00"] == 0:
            rospy.loginfo("Contour area is zero.")
            return None
        
        # Compute the center of the contour
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        
        # Draw a red circle at the center
        cv2.circle(image, (cX, cY), 10, (0, 0, 255), -1)
        
        # Draw grid lines
        height, width = image.shape[:2]
        cell_width = width // 3
        cell_height = height // 3
        for i in range(1, 3):
            cv2.line(image, (i * cell_width, 0), (i * cell_width, height), (255, 255, 255), 1)
            cv2.line(image, (0, i * cell_height), (width, i * cell_height), (255, 255, 255), 1)
        
        rospy.loginfo(f"Detected point at ({cX}, {cY})")
        return (cX, cY)

    # 1 2 3                 1,4,7번칸 우측 이동 의도 => 숫자 1 출력
    # 4 5 6 -> 화면분할 9칸   3,6,9번칸 좌측 이동 의도 => 숫자 2 출력       5번칸에 들어오면 => 정지 의도 숫자 5 출력
    # 7 8 9                 2, 8번칸  하강,상승 의도 => 숫자 3,4 출력 
    def center_mission(self, frame, lower_hsv, upper_hsv, window_name): #화면을 9등분해서 9칸중 어디에 빨간점이 찍히느냐에 따라 숫자를 출력
        # Convert the frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask using the provided HSV range
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # Apply the mask to get the filtered image
        colorExtract = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Find the center point
        detected_point = self.center_point(colorExtract)
        
        # Display the filtered image
        cv2.imshow(window_name, colorExtract)
        cv2.waitKey(1)  # To keep the window open for a short time
        
        if detected_point is None:
            rospy.loginfo("No point detected.")
            return None

        # Frame size
        height, width = frame.shape[:2]
        
        # Calculate cell width and height
        cell_width = width // 3
        cell_height = height // 3
        
        # Coordinates of the detected point
        cX, cY = detected_point
        
        # Determine the cell number based on the point's location
        row = cY // cell_height
        col = cX // cell_width
        cell_number = row * 3 + col + 1
        
        # Map cell number to specific values
        number_mapping = {
            1: 1, 2: 3, 3: 2,
            4: 1, 5: 5, 6: 2,
            7: 1, 8: 4, 9: 2
        }
        
        # Return the mapped number
        number = number_mapping.get(cell_number, None)
        rospy.loginfo(f"Detected number: {number}")
        return number
    
    def green_Goal_Drive(self): # mission C 녹색 골대 지나는 미션
        rospy.loginfo("MISSION: GOAL")
        t2 = rospy.get_time() #-> 갱신되면서 변할 시간 (변수) 
        rospy.loginfo(f"Current time: {t2}")
        # depth hold 0.5m 이동 후 천천히 전진 (5)초 -> 괄호 친 숫자는 튜닝해야되는 변수
        rospy.loginfo(f"static_flag_C: {self.static_flag_C}")
        
        # 미션을 처음 시작하는 경우 시간을 stop_t1으로 저장함
        if not self.static_flag_C:  # self.static_flag_C == False 대신 not self.static_flag_C 사용
            self.stop_t1_C = rospy.get_time() # start time -> 미션 시작하는 시간(상수)
            self.joy_msg.buttons[3] = 1 #lsh
            rospy.loginfo(f"Stop time set: {self.stop_t1_C}")
            self.static_flag_C = True
            rospy.loginfo(f"static_flag_C set to True")
            rospy.loginfo("MISSION: GOAL_start")
        
        rospy.loginfo(f"Elapsed time: {t2 - self.stop_t1_C}")
        
        # (2)초간 depth hold에서 아래로 -> 0.5m 맞추기
        if t2 - self.stop_t1_C <= 2:
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)
            self.joy_msg.axes[1] = 0.5 #lsh
            rospy.loginfo("MISSION: GOAL_down")
            
        # (5)초간 이전 각도로 주행 -> 골대 지나서 얼만큼 가는지 튜닝해야됨
        elif t2 - self.stop_t1_C <= 7: #시간은 누적으로 계산하세요
            rospy.loginfo("MISSION: GOAL_foward1")
            self.joy_msg.axes = [0] * len(self.joy_msg.axes)            
            self.joy_msg.axes[4] = 0.5 #lsh
            rospy.loginfo("MISSION: GOAL_foward2")
    
        else:
            self.mission_C = False
            self.static_flag_C = False  # 미션 종료 후 재설정
            rospy.loginfo("MISSION: GOAL_end")
            self.defaultDrive()  # 미션이 끝나면 기본 상태로 전환            
                            
        self.AUV_pub.publish(self.joy_msg) #lsh


    def defaultDrive(self): #기본 AUV 알고리즘에서 주행은 정지로 설정 -> 만약 아무런 커맨드를 안받았을 때는 정지해있도록
        rospy.loginfo("MISSION: Default Driving")
        # 모든 축을 0으로 초기화
        self.joy_msg.axes = [0] * len(self.joy_msg.axes)
        
        # depth hold 버튼을 활성화
        self.joy_msg.buttons[3] = 1
        
        # 기본 상태로 publish
        self.AUV_pub.publish(self.joy_msg)

#========================================================================================#
#========================================================================================#
#=========================== 조이스틱으로 미션 키기 위한 지정 함수들  ===========================#
#========================================================================================#
#========================================================================================#       
    def joystickCallback(self, data):
        self.joystick_axes = data.axes
        self.joystick_buttons = data.buttons
        
        # 배열의 길이를 로그로 출력
        rospy.loginfo(f"Axes length: {len(self.joystick_axes)}")
        rospy.loginfo(f"Buttons length: {len(self.joystick_buttons)}")
        
        # 나머지 로직
        if len(self.joystick_axes) > 7 and self.joystick_axes[7] == 1:
            rospy.loginfo("Received trigger to start mission_C.")
            self.mission_C = True
            self.mission_D_L = False
            self.mission_D_R = False        
            self.mission_E = False
            self.mission_F = False
            rospy.loginfo("Received trigger to start mission_D_L.")    
        elif len(self.joystick_axes) > 6 and self.joystick_axes[6] == 1:
            self.mission_C = False
            self.mission_D_L = True
            self.mission_D_R = False        
            self.mission_E = False
            self.mission_F = False
            rospy.loginfo("Received trigger to start mission_D_R.")
        elif len(self.joystick_axes) > 6 and self.joystick_axes[6] == -1:
            self.mission_C = False
            self.mission_D_L = False
            self.mission_D_R = True        
            self.mission_E = False
            self.mission_F = False
            rospy.loginfo("Received trigger to start mission_E.")
        elif len(self.joystick_axes) > 7 and self.joystick_axes[7] == -1:
            self.mission_C = False
            self.mission_D_L = False
            self.mission_D_R = False               
            self.mission_E = True
            self.mission_F = False  
            rospy.loginfo("Waiting trigger to start (waiting...).")


            # 필요에 따라 다른 버튼 처리 추가   
        
    def mainAlgorithm(self):
        rospy.loginfo("MAIN")
        # C. 녹색 골대 지나는 미션 
        if self.mission_C == True:
            self.green_Goal_Drive()
        
        # D. QR 코드 읽는 미션 L
        elif self.mission_D_L == True:
            self.QR_code_Drive_L()

        # D. QR 코드 읽는 미션 R
        elif self.mission_D_R == True:
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
    rospy.Timer(rospy.Duration(1.0/10.0), control.timerCallback) 
    rospy.spin()

if __name__ == "__main__":
    run()