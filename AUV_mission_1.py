import cv2
import sys
import numpy as np

def preprocess(img):
    # 원하는 색상 범위를 얻기 위해 이미지 전처리
    lower_hsv = np.array([91, 118, 74])  # HSV의 하한값
    upper_hsv = np.array([102, 255, 255])  # HSV의 상한값
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # BGR에서 HSV로 변환
    color_mask = cv2.inRange(hsvImg, lower_hsv, upper_hsv)  # 지정된 HSV 범위 내의 색상을 필터링하여 바이너리 마스크 생성
    return color_mask

# BlobDetector 설정
params = cv2.SimpleBlobDetector_Params()

# 면적으로 필터링
params.filterByArea = True
params.minArea = 90  # 최소 Blob 면적
params.maxArea = 500000000  # 최대 Blob 면적

# 원형도로 필터링
params.filterByCircularity = True
params.minCircularity = 0.1  # 최소 Blob 원형도

# 볼록도로 필터링
params.filterByConvexity = True
params.minConvexity = 0.03  # 최소 Blob 볼록도

# 관성으로 필터링
params.filterByInertia = True
params.minInertiaRatio = 0.0000001  # 최소 Blob 관성 비율

# Blob 간 거리 설정
params.minDistBetweenBlobs = 2000  # Blob 간 최소 거리

# 매개변수로 Detector 생성
detector = cv2.SimpleBlobDetector_create(params)

# 카메라 오픈
camera = cv2.VideoCapture(0)

while camera.isOpened():
    # 프레임 읽기
    retval, im = camera.read()

    # 이미지 전처리하여 원하는 색상 범위 얻기
    color_mask = preprocess(im)

    # 전처리된 이미지에서 Blob 감지
    keypoints = detector.detect(color_mask)

    # 감지된 Blob을 원본 이미지에 표시
    for k in keypoints:
        cv2.circle(im, (int(k.pt[0]), int(k.pt[1])), int(k.size / 2), (0, 0, 255), -1)  # Blob 주위에 채워진 원 그리기
        cv2.line(im, (int(k.pt[0]) - 20, int(k.pt[1])), (int(k.pt[0]) + 20, int(k.pt[1])), (0, 0, 0), 3)  # 수평선 그리기
        cv2.line(im, (int(k.pt[0]), int(k.pt[1]) - 20), (int(k.pt[0]), int(k.pt[1]) + 20), (0, 0, 0), 3)  # 수직선 그리기

    # 화면을 3등분 하는 수직선 그리기
    cv2.line(im, (im.shape[1] // 3, 0), (im.shape[1] // 3, im.shape[0]), (255, 255, 255), 2)
    cv2.line(im, (2 * im.shape[1] // 3, 0), (2 * im.shape[1] // 3, im.shape[0]), (255, 255, 255), 2)

    # 각 영역의 가운데에 작은 직사각형 그리기
    rect_width = im.shape[1] // 15  # 작은 직사각형의 폭
    rect_height = im.shape[0]  # 작은 직사각형의 높이
    for i in range(3):
        cv2.rectangle(im, (i * im.shape[1] // 3 + im.shape[1] // 6 - rect_width // 2, im.shape[0] // 2 - rect_height // 2), 
                      (i * im.shape[1] // 3 + im.shape[1] // 6 + rect_width // 2, im.shape[0] // 2 + rect_height // 2), 
                      (0, 255, 0), 2)  # 각 영역의 가운데에 작은 직사각형 그리기

    # Blob의 위치 확인 및 메시지 출력
    for k in keypoints:
        if im.shape[1] // 3 < k.pt[0] < 2 * im.shape[1] // 3:  # 가운데 영역
            if im.shape[0] // 3 < k.pt[1] < 2 * im.shape[0] // 3:  # 직사각형 내부에 있는지 확인
                print("center")
            else:
                print("middle")
        elif k.pt[0] < im.shape[1] // 3:  # 왼쪽 영역
            if im.shape[0] // 3 < k.pt[1] < 2 * im.shape[0] // 3:  # 직사각형 내부에 있는지 확인
                print("move right")
            else:
                print("left")
        elif k.pt[0] > 2 * im.shape[1] // 3:  # 오른쪽 영역
            if im.shape[0] // 3 < k.pt[1] < 2 * im.shape[0] // 3:  # 직사각형 내부에 있는지 확인
                print("move left")
            else:
                print("right")
        else:  # 가운데 영역 외의 영역
            print("outside rectangle")

    # 전처리된 이미지(이진 마스크) 표시
    cv2.imshow("Preprocessed Image (color Mask)", color_mask)

    # Blob이 감지된 원본 이미지 표시
    cv2.imshow("Output", im)

    k = cv2.waitKey(1) & 0xff
    if k == 27:  # ESC 키를 누르면 종료
        break

# 카메라 해제
camera.release()
cv2.destroyAllWindows()
