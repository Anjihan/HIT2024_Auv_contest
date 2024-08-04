#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar

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
        self.image_pub1 = rospy.Publisher("/camera1/image_raw", Image, queue_size=10)
        self.image_pub2 = rospy.Publisher("/camera2/image_raw", Image, queue_size=10)
        self.cap1 = cv2.VideoCapture(6)  # First camera
        self.cap2 = cv2.VideoCapture(0)  # Second camera

        # Define HSV ranges for each camera
        self.lower_hsv2 = (20, 100, 100)   # HSV range for the first camera
        self.upper_hsv2 = (30, 255, 255)
        self.lower_hsv1 = (20, 100, 100)   # HSV range for the second camera
        self.upper_hsv1 = (30, 255, 255)

        if not self.cap1.isOpened():
            rospy.logerr("Failed to open camera 6")
        if not self.cap2.isOpened():
            rospy.logerr("Failed to open camera 0")

        self.subscriber = rospy.Subscriber("/trigger_mission", String, self.trigger_callback)

        self.trigger_mission_a = False
        self.trigger_mission_b = False

    def center_mission(self, frame, lower_hsv, upper_hsv, window_name):
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

    def center_point(self, image):
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

    def camera_get(self):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()
        
        if ret1:
            img_msg1 = self.bridge.cv2_to_imgmsg(frame1, "bgr8")
            self.image_pub1.publish(img_msg1)
        else:
            rospy.logwarn("Failed to capture frame from camera 6.")
        
        if ret2:
            img_msg2 = self.bridge.cv2_to_imgmsg(frame2, "bgr8")
            self.image_pub2.publish(img_msg2)
        else:
            rospy.logwarn("Failed to capture frame from camera 0.")
            
        return ret1, frame1, ret2, frame2

    def mission_A(self):
        while not rospy.is_shutdown():
            ret1, frame1, ret2, frame2 = self.camera_get()
            
            if ret1:
                number1 = self.center_mission(frame1, self.lower_hsv1, self.upper_hsv1, "Camera 1")
                rospy.loginfo(f"Camera 1 detected number: {number1}")
                
            if ret2:
                number2 = self.center_mission(frame2, self.lower_hsv2, self.upper_hsv2, "Camera 2")
                rospy.loginfo(f"Camera 2 detected number: {number2}")
            # Check if either camera has detected the number 5
            if number1 == 5 and number2 == 5:
                rospy.loginfo("mission_A_complete")
                self.cap1.release()
                self.cap2.release()
                cv2.destroyAllWindows()                
                self.trigger_mission_a = False
                break
                    

    def QR_read_front(self):
        ret1, frame1, _, _ = self.camera_get()
        if ret1:
            barcode_data1 = self.qr_detector.read_frame(frame1)
            rospy.loginfo(f"Camera 1 QR 코드 데이터: {barcode_data1}")
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
            rospy.loginfo(f"Camera 2 QR 코드 데이터: {barcode_data2}")
            if barcode_data2:
                try:
                    return int(barcode_data2[0])
                except ValueError:
                    rospy.logwarn(f"Unable to convert QR code data '{barcode_data2[0]}' to integer.")
        return 0

    def mission_B(self):
        self.qr_front_result = self.QR_read_front()
        self.qr_top_result = self.QR_read_top()

        rospy.loginfo(f"QR Front Result: {self.qr_front_result}, QR Top Result: {self.qr_top_result}")

        # Check if the condition for mission_B is met
        if self.qr_front_result == 2 and self.qr_top_result == 1:
            rospy.loginfo("mission_B_complete")
            self.trigger_mission_b = False  # Reset the flag after execution
        else:
            rospy.loginfo("mission_B not complete: QR Front Result or QR Top Result does not match")

    def timerCallback(self, _event):
        try:
            self.mainAlgorithm()
        except Exception as e:
            rospy.logerr(f"Exception in timerCallback: {e}")

    def mainAlgorithm(self):
        if self.trigger_mission_a:
            self.mission_A()
            self.trigger_mission_a = False
        elif self.trigger_mission_b:
            self.mission_B()

    def trigger_callback(self, msg):
        if msg.data == "start_mission_A":
            rospy.loginfo("Received trigger to start mission_A.")
            self.trigger_mission_a = True
        elif msg.data == "start_mission_B":
            rospy.loginfo("Received trigger to start mission_B.")
            self.trigger_mission_b = True

def run():
    rospy.init_node("main_class_run")
    control = MainLoop()
    rospy.Timer(rospy.Duration(1.0/30.0), control.timerCallback) 
    rospy.spin()
    
if __name__ == '__main__':
    run()
