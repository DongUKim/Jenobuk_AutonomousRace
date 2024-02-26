
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2



class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')

        # Cvbridge 활성화
        self.bridge = CvBridge()

        # yellow lines subscribe
        # Pinkbot의 PiCamera로부터 CompressedImage를 전송받기 위한 ROS subscription 설정 
        self.subscription = self.create_subscription(
            CompressedImage, # 수신할 메시지 타입 지정
            'image_raw/compressed', # 구독할 토픽의 이름 지정
            self.CvImage, # 메시지가 도착했을때 실행할 callback 함수 지정
            5 ) # 메세지를 저장하는 버퍼의 크기 설정
        self.subscription  # prevent unused variable warning

        
        
        self.get_logger().info("Ready to detecting!")

    def range_RGB(self, r1, g1, b1, r2, g2, b2):
        self.r1 = r1
        self.g1 = g1
        self.b1 = b1
        self.r2 = r2
        self.g2 = g2
        self.b2 = b2

            
    def CvImage(self,msg):

        try:
            new_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Oops, CvBridge isn't working!")
        



        def remake(img):
            dst1 = cv2.inRange(img, (self.b1,self.g1,self.r1), (self.b2, self.g2, self.r2))
            # dst1 = cv2.resize(dst1, (960,540))
            return dst1
    
        edges = remake(new_img)

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
        cv2.imshow('image on your color range', lines)
        cv2.imshow('original', new_img)
        cv2.waitKey(1)
    
    

        
    


