import cv2
import sys
import numpy as np

def preprocess(img):
    lower_yellow = np.array([19, 105, 0])
    upper_yellow = np.array([42, 255, 255])
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsvImg, lower_yellow, upper_yellow)
    return yellow_mask

# Setup BlobDetector
params = cv2.SimpleBlobDetector_Params()

# Filter by Area.
params.filterByArea = True
params.minArea = 90  # 최소 부표 크기
params.maxArea = 500000000  # 최대 부표 크기

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1  # 최소 부표 원형도(얼마나 원에 가까운가)

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.03

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.0000001  # 최소 관성 비율

# Distance Between Blobs
params.minDistBetweenBlobs = 2000  # Blob간의 최소 거리(한개 이상 검출시에 제외 가능)

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

camera = cv2.VideoCapture(0)

while camera.isOpened():
    retval, im = camera.read()

    # Preprocess the image to get the desired color range
    yellow_mask = preprocess(im)

    # Detect blobs in the preprocessed image
    keypoints = detector.detect(yellow_mask)

    # Draw blobs on the original image
    for k in keypoints:
        cv2.circle(im, (int(k.pt[0]), int(k.pt[1])), int(k.size / 2), (0, 0, 255), -1)
        cv2.line(im, (int(k.pt[0]) - 20, int(k.pt[1])), (int(k.pt[0]) + 20, int(k.pt[1])), (0, 0, 0), 3)
        cv2.line(im, (int(k.pt[0]), int(k.pt[1]) - 20), (int(k.pt[0]), int(k.pt[1]) + 20), (0, 0, 0), 3)

    # Draw vertical lines to divide the screen into 3 parts
    cv2.line(im, (im.shape[1] // 3, 0), (im.shape[1] // 3, im.shape[0]), (255, 255, 255), 2)
    cv2.line(im, (2 * im.shape[1] // 3, 0), (2 * im.shape[1] // 3, im.shape[0]), (255, 255, 255), 2)

    # Display the preprocessed image (binary mask)
    cv2.imshow("Preprocessed Image (Yellow Mask)", yellow_mask)

    # Display the image with blobs
    cv2.imshow("Output", im)

    # Determine the position of keypoints
    for k in keypoints:
        if k.pt[0] < im.shape[1] // 3:
            print("left")
        elif k.pt[0] > 2 * im.shape[1] // 3:
            print("right")
        else:
            print("middle")

    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

camera.release()
cv2.destroyAllWindows()
