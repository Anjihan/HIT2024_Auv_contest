import cv2
import numpy as np

# Callback 함수
def nothing(x):
    pass

# 창 생성
cv2.namedWindow('Binary Image')

# 초기 Trackbar 위치
init_lower_h = 0
init_upper_h = 180
init_lower_s = 0
init_upper_s = 255
init_lower_v = 0
init_upper_v = 255

# Trackbar 생성
cv2.createTrackbar('Lower H', 'Binary Image', init_lower_h, 180, nothing)
cv2.createTrackbar('Upper H', 'Binary Image', init_upper_h, 180, nothing)
cv2.createTrackbar('Lower S', 'Binary Image', init_lower_s, 255, nothing)
cv2.createTrackbar('Upper S', 'Binary Image', init_upper_s, 255, nothing)
cv2.createTrackbar('Lower V', 'Binary Image', init_lower_v, 255, nothing)
cv2.createTrackbar('Upper V', 'Binary Image', init_upper_v, 255, nothing)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Trackbar에서 값 가져오기
    lower_h = cv2.getTrackbarPos('Lower H', 'Binary Image')
    upper_h = cv2.getTrackbarPos('Upper H', 'Binary Image')
    lower_s = cv2.getTrackbarPos('Lower S', 'Binary Image')
    upper_s = cv2.getTrackbarPos('Upper S', 'Binary Image')
    lower_v = cv2.getTrackbarPos('Lower V', 'Binary Image')
    upper_v = cv2.getTrackbarPos('Upper V', 'Binary Image')

    # HSV 이미지로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 색상 범위 설정
    lower_color = np.array([lower_h, lower_s, lower_v])
    upper_color = np.array([upper_h, upper_s, upper_v])

    # HSV 이미지에서 범위에 해당하는 영역을 이진화
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # 결과 출력
    cv2.imshow('Original', frame)
    cv2.imshow('Binary Image', mask)

    # 종료 조건
    if cv2.waitKey(1) & 0xff == 27:
        break

cap.release()
cv2.destroyAllWindows()
