#!/usr/bin/env python3

import cv2
import socket
import numpy as np

# B PC의 IP와 포트를 설정합니다.
B_IP = '127.0.0.1'  # 수신 측 IP 주소
B_PORT = 8080  # 포트 번호
B_PORT_2 = 8081  # 두 번째 카메라 데이터를 보낼 포트 번호

# UDP 소켓을 생성합니다.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 첫 번째 카메라를 시작합니다.
cap = cv2.VideoCapture(4)

# 두 번째 카메라를 시작합니다.
cap1 = cv2.VideoCapture(6)

while True:
    # 첫 번째 카메라로부터 이미지를 캡처합니다.
    ret, frame = cap.read()
    if not ret:
        break

    # 첫 번째 카메라 이미지 전송
    encoded, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    sock.sendto(buffer, (B_IP, B_PORT))

    # 두 번째 카메라로부터 이미지를 캡처합니다.
    ret1, frame1 = cap1.read()
    if not ret1:
        break

    # 두 번째 카메라 이미지 전송
    encoded1, buffer1 = cv2.imencode('.jpg', frame1, [cv2.IMWRITE_JPEG_QUALITY, 50])
    sock.sendto(buffer1, (B_IP, B_PORT_2))

    # 송신 측에서도 이미지를 확인할 수 있도록 표시
    cv2.imshow('First Camera Image', frame)
    cv2.imshow('Second Camera Image', frame1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원을 해제합니다.
cap.release()
cap1.release()
sock.close()
cv2.destroyAllWindows()
