#!/usr/bin/env python3

import cv2
import imutils
import pyzbar.pyzbar as pyzbar
import rospy

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

def qr_code_reader():
    rospy.init_node('qr_code_reader', anonymous=True)
    qr_detector = QRdetect()

    cap = cv2.VideoCapture(6)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    if not cap.isOpened():
        rospy.loginfo("카메라를 열 수 없습니다.")
        return

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            barcode_data = qr_detector.read_frame(frame)
            if barcode_data:
                rospy.loginfo(f"QR 코드 데이터: {barcode_data}")
            if cv2.waitKey(1) == 27:  # ESC 키를 누르면 종료
                break
        else:
            rospy.loginfo("프레임을 읽을 수 없습니다.")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    qr_code_reader()
