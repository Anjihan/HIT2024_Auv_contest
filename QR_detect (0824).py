#!/usr/bin/env python3

import cv2
import pyzbar.pyzbar as pyzbar
import rospy

class QRdetect:
    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.valid_codes = {"LH1", "LL1", "RH1", "RL1", "LH2", "LL2", "RH2", "RL2"}  # 인식할 유효한 QR 코드 목록

    def read_frame(self, frame):
        # QR 코드 정보 디코딩
        barcodes = pyzbar.decode(frame)
        recognized_codes = []
        for barcode in barcodes:
            barcode_info = barcode.data.decode('utf-8')
            rospy.loginfo(f"QR 코드 인식됨: {barcode_info}")  # 인식된 QR 코드 정보 출력
            if barcode_info in self.valid_codes:
                recognized_codes.append(barcode_info)
        return recognized_codes

def qr_code_reader():
    rospy.init_node('qr_code_reader', anonymous=True)
    qr_detector = QRdetect()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    if not cap.isOpened():
        rospy.loginfo("카메라를 열 수 없습니다.")
        return

    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # rospy.loginfo("프레임을 읽어왔습니다.")
            recognized_codes = qr_detector.read_frame(frame)
            if recognized_codes:
                rospy.loginfo(f"유효한 QR 코드 인식됨: {recognized_codes}")
            else:
                pass # rospy.loginfo("유효한 QR 코드가 인식되지 않았습니다.")
        else:
            rospy.loginfo("프레임을 읽을 수 없습니다.")
            break

        if cv2.waitKey(1) == 27:  # ESC 키를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    qr_code_reader()
