import sys
import geometry_msgs.msg
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from rclpy.node import Node
import threading
from std_msgs.msg import Int32,String
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult

from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose



class RunPinkbot(Node):
    def __init__(self):
        super().__init__('run_pinkbot')

        # SLAM을 이용해 로봇의 현재 위치를 파악하기 위한 sub
        self.subscription_position = self.create_subscription(
            PoseWithCovarianceStamped, # 수신할 메시지 타입 지정
            'amcl_pose', # 구독할 토픽의 이름 지정
            self.check_position, # 메시지가 도착했을때 실행할 callback 함수 지정
            5 ) # 메세지를 저장하는 버퍼의 크기 설정
        self.subscription_position  # prevent unused variable warning


        # lane_detect에 따라 로봇의 구동을 결정하기 위한 state topic sub
        self.subscription_lane = self.create_subscription(
            String, 
            'state',
            self.lane_detect, 
            5 ) 
        self.subscription_lane  

        # lane_detect을 못할 경우 SLAM으로 로봇을 이동시키기 위한 slam topic sub
        self.subscription_slam = self.create_subscription(
            String, 
            'mode', 
            self.mode, 
            5 ) 
        self.subscription_slam  

        self.get_logger().info("Loading...")

        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        
        # 결과값 설정
        self.result = self.nav.getResult()


    # amcl_pose를 sub해 현재 로봇의 좌표를 체크
    def check_position(self,msg):
        print(msg.data)

    # pi camera2 를 이용해 lane_detect를 하여 로봇을 조종하거나 slam 활성화
    def lane_detect(self, msg):
        self.state = msg.data
        print("Now state is ", self.state)
        node = rclpy.create_node('teleop_twist_keyboard')
        
         # parameters
        stamped = node.declare_parameter('stamped', False).value
        frame_id = node.declare_parameter('frame_id', '').value
        if not stamped and frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        if stamped:
            TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            TwistMsg = geometry_msgs.msg.Twist

        pub = node.create_publisher(TwistMsg, '/base_controller/cmd_vel_unstamped', 10)

        speed = 0.1
        turn = 0.5
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0
        status = 0.0

        twist_msg = TwistMsg()
        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = node.get_clock().now().to_msg()
            twist_msg.header.frame_id = frame_id
        else:
            twist = twist_msg
        try:

            Node.subscription

            while True:
                print("debug 1")
                if self.mode == "lane_detect":

                    # Lane Detecting Mode state 1
                    if self.state == "Go":
                        x = 1.0
                        y = 0.0
                        z = 0.0
                        th = 0.0


                    # Right
                    elif self.state == "Right":
                        x = 0.7
                        y = 0.0
                        z = 0.0
                        th = -0.8

                    # Left
                    elif self.state == "Left":
                        x = 0.7
                        y = 0.0
                        z = 0.0
                        th = 0.8

                    else:
                        pass

                    if stamped:
                        twist_msg.header.stamp = node.get_clock().now().to_msg()
                    speed = 0.1
                    twist.linear.x = x * speed *(2/3)
                    twist.linear.y = y * speed *(2/3)
                    twist.linear.z = z * speed *(2/3)
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = th * turn
                    pub.publish(twist_msg)
                    print("debug 2")
                 # Slam mode Change
                elif self.mode == "slam":
                    # 일시 정지
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    th = 0.0
                    
                    print("\nReady to SLAM Mode\n")
                    self.goal_poses.header.stamp = self.nav.get_clock().now().to_msg()
                    print("debug 4")
                    if stamped:
                        twist_msg.header.stamp = node.get_clock().now().to_msg()

                    twist.linear.x = x * speed
                    twist.linear.y = y * speed
                    twist.linear.z = z * speed
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = th * turn
                    pub.publish(twist_msg)
                    
                    time.sleep(3)
                    slam = slam(self)
                    slam

                    # 만약 이 코드가 정상적으로 실행이 되어서, 미션 수행에 대한 루트의 경우의수를 따져야할때 추가
                    # route = int(input("Please select location number between 1 & 4 : "))
                   
                else:
                    print("Horrible trouble")
        except Exception as e:
            print(e)

        finally:
            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            pub.publish(twist_msg)
            rclpy.shutdown()
                

    def mode(self, msg):
            
        self.mode = msg.data

    
    def slam(self):
        self.goal_poses = PoseStamped()
        self.goal_poses.header.frame_id = 'map'
        self.goal_poses.header.stamp = self.nav.get_clock().now().to_msg()


        # 교차로의 중앙지점 way_point
        self.goal_poses.pose.position.x = -0.21012544697137217
        self.goal_poses.pose.position.y = -0.24670609197710971
        self.goal_poses.pose.position.z = 0.0
        self.goal_poses.pose.orientation.x = 0.0
        self.goal_poses.pose.orientation.y = 0.0
        self.goal_poses.pose.orientation.z = 0.999
        self.goal_poses.pose.orientation.w = 0.00833

        print("Now Pinkbot's goal:\n", 
            "x = ", self.goal_poses.pose.position.x,
            "\n y = ", self.goal_poses.pose.position.y, 
            "\n z = ", self.goal_poses.pose.position.z)
                        # 이동
        self.nav.goToPose(self.goal_poses)
        time.sleep(0.5)
        print("\nslam start\n")
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            # print("feedback =", feedback)
            if feedback:
                print("Distance remaining: " + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.'
        )
        if self.nav.isTaskComplete():
            self.state = "lane_detect"
        
        



def main(args = None):

    rclpy.init(args=args)
    run_pinkbot = RunPinkbot()
    run_pinkbot
    rclpy.shutdown()


    


if __name__ == '__main__':
    main()
