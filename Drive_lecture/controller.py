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


    def drive_test(x,speed,theta,time_action):
        rclpy.init()
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

    
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = node.get_clock().now().to_msg()
            twist_msg.header.frame_id = frame_id
        else:
            twist = twist_msg
            
        start_time = time.time()
        
        # print(run_time)
        while True:
            run_time = time.time() - start_time
            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = theta * speed

            # 속도와 방향에 대한 정보를 pub
            pub.publish(twist_msg)
            print(run_time)

            # 로봇 구동
            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()
            
            # while문 동작시간이 time_action 시간보다 넘어가면 종료
            # + 0.5는 ros로 송수신하면서 실제로 모터가 작동되는 시간이 time_action보다 줄어들기 때문에, 이를 보정하기 위해 임의로 입력한 수치
            if (run_time > time_action +0.5):
                break
        rclpy.shutdown()

            
            
            


        