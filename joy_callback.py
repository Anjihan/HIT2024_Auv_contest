#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

class MainLoop:
    def __init__(self):
        rospy.init_node('auv_mission_node', anonymous=True)
        
        # 조이스틱 데이터 구독
        self.joystick_sub = rospy.Subscriber("/joy", Joy, self.joystickCallback)
        
        # 조이스틱 축 및 버튼 상태 초기화
        self.joystick_axes = []
        self.joystick_buttons = []
        
        # 미션 상태 초기화
        self.mission_C = False
        self.mission_D_L = False
        self.mission_D_R = False
        self.mission_E = False
        self.mission_F = False
        
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8  # 예시: 8개의 축
        self.joy_msg.buttons = [0] * 12  # 예시: 12개의 버튼
        self.AUV_pub = rospy.Publisher("/joy", Joy, queue_size=10)

        self.stop_t1_C = 0.0
        self.static_flag_C = False

    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
        except Exception as e:
            rospy.logerr(f"Exception in timerCallback: {e}")

    def joystickCallback(self, data):
        self.joystick_axes = data.axes
        self.joystick_buttons = data.buttons
        
        # rospy.loginfo(f"Axes length: {len(self.joystick_axes)}")
        # rospy.loginfo(f"Buttons length: {len(self.joystick_buttons)}")
        
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
        
    def mainAlgorithm(self):
        rospy.loginfo("MAIN")
        if self.mission_C:
            self.green_Goal_Drive()
        else:
            self.defaultDrive()
            
    def defaultDrive(self):
        rospy.loginfo("MISSION: Default Driving")
        # 모든 축을 0으로 초기화
        self.joy_msg.axes = [0] * len(self.joy_msg.axes)

        # depth hold 버튼을 활성화
        self.joy_msg.buttons[3] = 1

        # 기본 상태로 publish
        self.AUV_pub.publish(self.joy_msg)
                      
    def green_Goal_Drive(self):
        rospy.loginfo("MISSION: GOAL")
        t2 = rospy.get_time()
        # rospy.loginfo(f"Current time: {t2}")
        # rospy.loginfo(f"static_flag_C: {self.static_flag_C}")
        
        if not self.static_flag_C:
            self.stop_t1_C = rospy.get_time()
            # rospy.loginfo(f"Stop time set: {self.stop_t1_C}")
            self.static_flag_C = True
            # rospy.loginfo(f"static_flag_C set to True")
            # rospy.loginfo("MISSION: GOAL_start")
        
        rospy.loginfo(f"Elapsed time: {t2 - self.stop_t1_C}")
        
        if t2 - self.stop_t1_C <= 2:
            self.joy_msg.buttons[3] = 1
            self.joy_msg.axes[1] = 0.5
            rospy.loginfo("MISSION: GOAL_down")
        elif t2 - self.stop_t1_C <= 7:
            rospy.loginfo("MISSION: GOAL_foward1")
            self.joy_msg.axes[4] = 0.5
            rospy.loginfo("MISSION: GOAL_foward2")
        else:
            self.mission_C = False
            self.static_flag_C = False  # 미션 종료 후 재설정
            rospy.loginfo("MISSION: GOAL_end")
            self.defaultDrive()  # 미션이 끝나면 기본 상태로 전환
                            
        # rospy.loginfo(f"Publishing joy data: buttons={self.joy_msg.buttons}, axes={self.joy_msg.axes}")
        self.AUV_pub.publish(self.joy_msg)

def run():
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback)
    rospy.spin()

if __name__ == "__main__":
    run()
