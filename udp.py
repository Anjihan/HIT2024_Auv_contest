#!/usr/bin/env python3

import cv2
import socket
import numpy as np

# B의 IP와 포트를 설정합니다.
B_IP = '192.168.123.5'
B_PORT = 8080

# 루프백 주소에 대한 UDP 소켓을 생성합니다.
#loopback_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#loopback_sock.bind(('127.0.0.1', B_PORT))

# UDP 소켓을 생성하고 B의 IP와 포트에 바인드합니다.
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((B_IP, B_PORT))

while True:
    # UDP 소켓으로부터 데이터를 수신합니다.
    data, addr = sock.recvfrom(65536) # 버퍼 크기를 지정합니다.
    # 데이터를 이미지로 디코딩합니다.
    npdata = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)

    if frame is not None:
        # 이미지를 화면에 표시합니다.
        cv2.imshow('Received Image', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 이미지 데이터를 루프백 주소로 다시 전송합니다.
        #loopback_sock.sendto(data, ('127.0.0.1', B_PORT))

# 자원을 해제합니다.
cv2.destroyAllWindows()
sock.close()
loopback_sock.close()

