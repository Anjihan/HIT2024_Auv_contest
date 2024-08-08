import cv2

def find_available_cameras():
    available_cameras = []
    for i in range(0, 101):  # 0부터 100까지 반복
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera {i} is available.")
            available_cameras.append(i)
            cap.release()  # 카메라 리소스 해제
        else:
           pass # print(f"Camera {i} is not available.")
    
    return available_cameras

# 사용 가능한 카메라 인덱스를 찾습니다.
cameras = find_available_cameras()
print("Available cameras:", cameras)
