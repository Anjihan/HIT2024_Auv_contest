import cv2
import pyrealsense2 as rs
import numpy as np
# -> 거리쓰는법, 색깔쓰는법
# 
# -> 제어=이동 -> rostopic //
# -> IMU -> // 180 + 270 - 90
# |=========|
# |         |
# |         |
class Depth_Camera():

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.align_to = None

        self.fps = 30
        
        context = rs.context()
        connect_device = None
        if context.devices[0].get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device = context.devices[0].get_info(rs.camera_info.serial_number)

        print(" > Serial number : {}".format(connect_device))
        self.config.enable_device(connect_device)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)

    def __del__(self):
        print("Collecting process is done.\n")

    def execute(self):
        print('Collecting depth information...')
        try:
            self.pipeline.start(self.config)
        except:
            print("There is no signal sended from depth camera.")
            print("Check connection status of camera.")
            return
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        try:
            while True:

                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                depth_info = depth_frame.as_depth_frame()

                x, y = 320, 240
                print("Depth_0: ", round((depth_info.get_distance(10, y) * 100), 2), "cm")
                print("Depth_1 : ", round((depth_info.get_distance(320, y) * 100), 2), "cm")
                print("Depth_2: ", round((depth_info.get_distance(630, y) * 100), 2), "cm")


                color_image = np.asanyarray(color_frame.get_data())
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                color_image = cv2.circle(color_image, (x, y), 2, (0, 0, 255), -1)
                color_image = cv2.circle(color_image, (630, y), 2, (0, 0, 255), -1)
                color_image = cv2.circle(color_image, (10, y), 2, (0, 0, 255), -1)                
                cv2.imshow('RealSense', color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipeline.stop()
        print("┌──────────────────────────────────────┐")
        print('│ Collecting of depth info is stopped. │')
        print("└──────────────────────────────────────┘")

if __name__ == "__main__":
    depth_camera = Depth_Camera()
    depth_camera.execute()