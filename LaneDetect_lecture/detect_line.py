
import rclpy

import time
from rclpy.node import Node
import threading
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.duration import Duration
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped


class LaneDetect(Node):
            
    def init(self, r1, g1, b1, r2, g2, b2):
        

        self.r1 = r1
        self.g1 = g1
        self.b1 = b1
        self.r2 = r2
        self.g2 = g2
        self.b2 = b2

        self.bridge = CvBridge()

        # Pinkbot의 PiCamera로부터 CompressedImage를 전송받기 위한 ROS subscription 설정 
        self.subscription = self.create_subscription(
            CompressedImage, # 수신할 메시지 타입 지정
            'image_raw/compressed', # 구독할 토픽의 이름 지정
            self.CvImage, # 메시지가 도착했을때 실행할 callback 함수 지정
            5 ) # 메세지를 저장하는 버퍼의 크기 설정
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Ready to detecting!")

        spinner = threading.Thread(target=rclpy.spin)
        spinner.start()

    def lane_detect(self, msg, r1, g1, b1, r2, g2, b2):

        # init 함수를 실행
        initalize = self.init(self, r1, g1, b1, r2, g2, b2)

        rclpy.init()

        def CvImage(self, msg):
            try:
                new_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError:
                print("Oops, CvBridge isn't working!")

            # 특정 영역의 색상만을 추출한다.
            # ipynb 코드에서 r1,2 g1,2 b1,2 에 대한 값을 입력해야함.
            def color_detect(self, img):
                dst1 = cv2.inRange(img, (self.b1,self.g1,self.r1), (self.b2, self.g2, self.r2))
                return dst1
        
            edges = color_detect(new_img)

            def region_of_interest(img, vertices):

                mask = np.zeros_like(img)

                if len(img.shape) > 2:
                    channel_count = img.shape[2]
                    ignore_mask_color = (255,) * channel_count
                else:
                    ignore_mask_color = 255

                cv2.fillPoly(mask, vertices, ignore_mask_color)

                masked_image = cv2.bitwise_and(img, mask)
                
                return masked_image

            new_imshape = new_img.shape

            height = new_imshape[0]
            width = new_imshape[1]

            vertices = np.array([[(width/4, height),
                                    (width/4,0),
                                    (width*3/4,0),
                                    (width*3/4, height)]], dtype=np.int32)

            mask = region_of_interest(edges, vertices)

            def draw_lines(img, lines, color=[125, 125, 255], thickness=5):
                avg_x = []
                avg_y = []

                if lines is not None:

                    for line in lines:
                        for x1, y1, x2, y2 in line:
                            cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                            avg_x_val = (x1+x2)/2
                            avg_y_val = (y1+y2)/2
                            avg_x.append(avg_x_val)
                            avg_y.append(avg_y_val)
                
                    total_avg_x = sum(avg_x) / len(avg_x) 
                    total_avg_y = sum(avg_y) / len(avg_y) 

                    
                
                        
            def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
                lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
                line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
                draw_lines(line_img, lines)

                return line_img

            rho = 2
            theta = np.pi/180
            threshold = 90
            min_line_len = 120
            max_line_gap = 150

            lines = hough_lines(mask, rho, theta, threshold, min_line_len, max_line_gap)


            # def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
            #     return cv2.addWeighted(initial_img, a, img, b,c)

            # lines_edges = weighted_img(lines,new_img, a=0.8, b=1., c=0.)
            cv2.imshow('y_image_raw/compressed', lines)
            cv2.imshow('original', new_img)
            cv2.waitKey(1)
            rclpy.shutdown()
            spinner.join()
    
    

    


