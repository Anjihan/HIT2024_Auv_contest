import cv2
import imutils
import pyzbar.pyzbar as pyzbar
import numpy as np

class qr_code_reader():
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    def __init__(self):
        pass

    def read_frame(frame):
        try:
            # 코드 정보 디코딩
            barcodes = pyzbar.decode(frame)
            barcode_data = []
            # 하나씩
            for barcode in barcodes:
                # 코드 rect정보
                x, y, w, h = barcode.rect
                # 코드 데이터 디코딩
                barcode_info = barcode.data.decode('utf-8')
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, barcode_info, (x, y - 20), font, 0.5, (0, 0, 255), 1)
                # QR 코드 데이터 리스트에 추가
                barcode_data.append(barcode_info)
            return frame, barcode_data
        except Exception as e:
            print(f"Error in read_frame: {e}")
            return frame, []

    try:
        cap = cv2.VideoCapture(0)
        # 해상도 변경
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            raise Exception("카메라 안댐")
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # 프레임 Resize
                frame = imutils.resize(frame, width=1920)
                frame, barcode_data = read_frame(frame)
                
                # QR 코드 데이터 출력
                if barcode_data:
                    print("QR 코드 데이터:", barcode_data)
                
                # 화면에 프레임 출력
                cv2.imshow("QR Code Reader", frame)
                
                # 루프 종료
                if cv2.waitKey(1) == 27:
                    break
            else:
                print("프레임 없음")
                break
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
