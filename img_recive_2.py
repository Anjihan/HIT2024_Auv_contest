#!/usr/bin/env python3

import cv2
import socket
import numpy as np

# B PC의 IP와 포트를 설정합니다.
B_IP = '127.0.0.1'  # 수신 측 IP 주소
B_PORT = 8080  # 첫 번째 카메라 데이터를 받을 포트 번호
B_PORT_2 = 8081  # 두 번째 카메라 데이터를 받을 포트 번호

# UDP 소켓을 생성하고 B PC의 IP와 포트에 바인드합니다.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((B_IP, B_PORT))

sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2.bind((B_IP, B_PORT_2))

while True:
    # 첫 번째 카메라로부터 데이터를 수신합니다.
    data, addr = sock.recvfrom(65536)
    npdata = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)

    # 두 번째 카메라로부터 데이터를 수신합니다.
    data1, addr1 = sock2.recvfrom(65536)
    npdata1 = np.frombuffer(data1, dtype=np.uint8)
    frame1 = cv2.imdecode(npdata1, cv2.IMREAD_COLOR)

    # 각 이미지가 제대로 수신되었는지 확인합니다.
    if frame is not None:
        cv2.imshow('Received First Camera Image', frame)
    
    if frame1 is not None:
        cv2.imshow('Received Second Camera Image', frame1)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원을 해제합니다.
cv2.destroyAllWindows()
sock.close()
sock2.close()
