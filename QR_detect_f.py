#!/usr/bin/env python3

import cv2
import pyzbar.pyzbar as pyzbar
import rospy
import sys

class QRdetect:
    def __init__(self):
        self.code_mapping = {
            "LH1": 1, "LL1": 2, "LH2": 3, "LL2": 4,
            "RH1": 5, "RL1": 6, "RH2": 7, "RL2": 8
        }

    def read_frame(self, frame):
        # QR 코드 정보 디코딩
        barcodes = pyzbar.decode(frame)
        recognized_codes = []
        for barcode in barcodes:
            barcode_info = barcode.data.decode('utf-8').strip()
            if barcode_info in self.code_mapping:
                code_number = self.code_mapping[barcode_info]
                recognized_codes.append(code_number)
                rospy.loginfo(f"QR 코드 숫자 인식됨: {code_number}")  # 숫자만 출력
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

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            recognized_codes = qr_detector.read_frame(frame)
            for code in recognized_codes:
                if code == 1:  # 1을 인식했을 때 종료
                    rospy.loginfo("코드 1이 인식되어 프로그램을 종료합니다.")
                    cap.release()
                    cv2.destroyAllWindows()
                    rospy.signal_shutdown("코드 1 인식으로 인한 종료")
                    sys.exit(0)
        else:
            rospy.loginfo("프레임을 읽을 수 없습니다.")
            break

        if cv2.waitKey(1) == 27:  # ESC 키를 누르면 종료
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    qr_code_reader()
