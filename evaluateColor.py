import cv2
import numpy as np

class Evaluator:
    def __init__(self):
        
        pass

    def evaluate(self, originalImage):

        lower_yellow = np.array([19, 105, 0])
        upper_yellow = np.array([42, 255, 255])
        hsvImg = cv2.cvtColor(originalImage, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsvImg, lower_yellow, upper_yellow)
        # 부표 색상으로 튜닝

        return yellow_mask
    
    def is_buoy(self, img):
        mask = self.preprocess(img)

        detector = cv2.SimpleBlobDetector_create() # 개체 검출 객체 생성
        params = cv2.SimpleBlobDetector_Params()   # 파라미터 설정 객체(크기, 원형도, 관성 비율)

        # Area
        params.filterByArea = True
        params.minArea = 90  # 최소 부표 크기
        params.maxArea = 500000000  # 최대 부표 크기

        # Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1  # 부표 최소 원형도

        # Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.03

        # Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.0000001  # 최소 관성 비율

        # Blob 간의 거리 설정
        params.minDistBetweenBlobs = 100000

        # 파라미터
        detector = cv2.SimpleBlobDetector_create(params)

        keypoints = detector.detect(mask) # mask된 이미지에서 부표의 위치 및 특성 검출
        
        ret = False
        detected_points = [False, False]
        for k in keypoints:
            ret = True
            detected_points = (int(k.pt[0]), int(k.pt[1]))
            return ret, detected_points
            
        return ret, detected_points