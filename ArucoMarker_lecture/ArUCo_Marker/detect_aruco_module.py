import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.duration import Duration
from cv_bridge import CvBridge, CvBridgeError

import cv2
import sys

class ArucoDetect(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        
        self.ARUCO_DICT= {
                    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
                    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
                    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
                    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
                    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
                    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
                    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
                    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
                    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
                    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
                    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
                    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
                    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
                    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
                    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
                    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
                    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
                    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
                    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
                    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
                    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
                    }
        

        self.aruco_subscriber = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.CvImage,
            10 )

        self.aruco_publisher = self.create_publisher(
            Int32MultiArray,
            '/detect_aruco_num',
            10
        )  
 
        self.bridge = CvBridge()

    def CvImage(self, msg):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("CvBridge is not working, check plz")


        def aruco_display(self, corners, ids, rejected, image):
            
            if len(corners) > 0:
                # ArUco 마커의 ID 목록을 1차원 배열로 변환
                ids = ids.flatten()

                for (markerCorner, markerID) in zip(corners, ids):
                    # 마커 모서리 추출
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    # 마커 모서리 그리기
                    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    # 마커 중심점 그리기
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                    # 마커 ID 번호 쓰기
                    cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10),
                                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return image

        def aruco_detect(self, camera, video, aruco_type):
            # ArUco 타입이 맞지 않으면 종료하기
            if self.ARUCO_DICT.get(aruco_type, None) is None :
                print(f"ArUCo tag type '{aruco_type}' is not supproted")
                sys.exit(0)

            aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[aruco_type])
            aruco_params = cv2.aruco.DetectorParameters_create()

            h, w, _ = self.image.shape

            # 작업속도를 위한 이미지 처리
            width = 100
            height = int(width * (h / w))
            frame = self.image
            # frame = cv2.resize(self.image, (width, height), interpolation=cv2.INTER_CUBIC)
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict,
                                                              parameters=aruco_params)

            # 마커가 인식된 경우에만 Print 하기
            if ids is not None and len(ids) > 0:
                for markerID in ids :
                    print("Detect ArUco marker ID: {}".format(markerID))

                    # detect_aruco_num Topic으로 보내기
                    msg = Int32MultiArray()
                    msg.data = [int(markerID)]
                    self.aruco_publisher.publish(msg)

            # 마커 정보 화면에 그리기
            detected_markers = aruco_display(self, corners, ids, rejected, frame)
            return detected_markers
        
        last_iamge = aruco_detect(self, camera=True, video=None, aruco_type="DICT_4X4_50")

        cv2.imshow("image_raw/compressed", last_iamge)
        cv2.waitKey(1)

def aruco_main(args=None):

    rclpy.init(args=args)
    
    aruco_detect = ArucoDetect()
    rclpy.spin(aruco_detect)
    rclpy.shutdown()


if __name__ == "__main__":
    aruco_main()
    


