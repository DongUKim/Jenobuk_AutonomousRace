
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
    def __init__(self):
        super().__init__('lane_detect')


        # 로봇 구동 시작할때의 초기값
        self.state = 0
        self.mode = 0

        # topic publish
        self.publisher_state = self.create_publisher(Int32, 'state', 10)
        self.publisher_mode = self.create_publisher(Int32, 'mode', 10)

        # publish timer
        self.timer = self.create_timer(1, self.publish_state)
        self.timer = self.create_timer(1, self.publish_mode)

        # Cvbridge 활성화
        self.bridge = CvBridge()

        # yellow lines subscribe
        # Pinkbot의 PiCamera로부터 CompressedImage를 전송받기 위한 ROS subscription 설정 
        self.subscription = self.create_subscription(
            CompressedImage, # 수신할 메시지 타입 지정
            'image_raw/compressed', # 구독할 토픽의 이름 지정
            self.y_CvImage, # 메시지가 도착했을때 실행할 callback 함수 지정
            5 ) # 메세지를 저장하는 버퍼의 크기 설정
        self.subscription  # prevent unused variable warning

        
        
        self.get_logger().info("Ready to detecting!")

    # state는 로봇이 구동해야하는 방향을 지칭함(ex.Go, Right, Left)
    def publish_state(self):
        msg = Int32()
        msg.data = self.state
        self.publisher_state.publish(msg)
        self.get_logger().info(f"Pinkbot {self.state}")

    def publish_mode(self):
        msg = Int32()
        msg.data = self.mode
        self.publisher_mode.publish(msg)
        self.get_logger().info(f"Pinkbot {self.mode}")
    # def publish_mode(self):
    #     mode = input("Please select location both lane_detect or slam : ")  # 입력된 숫자를 받음
    #     msg = String()
    #     msg.data = mode
    #     self.publisher_mode.publish(msg)
    #     self.get_logger().info(f"You selected : {mode} Mode")
            
    def y_CvImage(self,msg):

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Oops, CvBridge isn't working!")
        
        # 영상에서 특정 영역의 색상만 추출시킨다.
        def bird(img):
            imshape = img.shape

            # width = imshape[1]
            height = imshape[0]

            pts1 = np.float32([[60, 200],[60, height],[340,200],[340, height]]) 

            # 좌표의 이동점
            pts2 = np.float32([[10,10],[10,1000],[1000,10],[1000,1000]])

            M = cv2.getPerspectiveTransform(pts1, pts2)

            img = cv2.warpPerspective(img, M, (1100,1000))

            b, g, r = cv2.split(img)

            new_img = cv2.merge((b,g,r))

            return new_img

        new_img = bird(img)


        def remake(img):
            dst1 = cv2.inRange(img, (0,160,160), (150, 255, 255))
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

                ##### Drive, Curve Algorithm #####

                # state 
                # 3 = [default state]

                # 0 = Go
                # 1 = Left
                # 2 = Right

                    

                # Go 0
                if (width/2 - 150) <= total_avg_x <= (width/2 + 150):
                    
                    
                    # print(" GO_moter_output up ! ") 
                    if (self.state == 0):
                        if (self.mode == 0):
                            pass
                        else:
                            self.mode = 0    
                                                      
                    else:
                        self.state = 0 
                                           
                
                # Left  1
                if total_avg_x < (width/2 - 150):
                    # print("average x : ", total_avg_x)
                    # print(" Left moter_output up ! ")
                    if (self.state == 1):
                        if (self.mode == 0):
                            pass
                        else:
                            self.mode = 0   
                                                      
                    else:
                        self.state = 1 

                # Right 2
                if total_avg_x > (width/2 + 150):
                    # print("average x : ", total_avg_x)
                    # print(" Right moter_output up ! ")
                    if (self.state == 2):
                        if (self.mode == 0):
                            pass
                        else:
                            self.mode = 0    
                                                      
                    else:
                        self.state = 2

            # SLAM Mode Change Algorizm
            
            else : 
                self.mode = 1 

                
            
                    
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
        cv2.imshow('original', img)
        cv2.waitKey(1)
    
    

            
            

def main(args=None):
    # Gazebo Setup!
    rclpy.init(args=args)
    
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    


