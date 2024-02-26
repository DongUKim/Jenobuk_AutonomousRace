import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

from ultralytics import YOLO

class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detect')
        
        # 사용할 파라미터를 불러온다    
        self.declare_parameter('rgb_topic', 'image_raw/compressed')
        self.declare_parameter('conf', 0.7)
        self.declare_parameter("device", "cpu") 
        # 파라미터의 값을 저장한다
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.conf = self.get_parameter('conf').value
        self.device = self.get_parameter('device').value

        self.colcor_subscriber = self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.color_image_callback,
            10
        )
    
        self.detect_publisher = self.create_publisher(
            Int32MultiArray,
            '/detect_object',
            10
        )

        self.cv_bridge = CvBridge()
    
        # 학습한 YOLO 모델 불러오기
        # 개인이 학습한 best.pt 가져와서 사용하기
        self.model = YOLO('yolov8n.pt')
        #self.model = YOLO('second_human.pt')
        #self.model = YOLO('stop_best.pt')
    
    def color_image_callback(self, msg):
        color_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(color_image, conf=self.conf, device=self.device) #detect
        result = results[0].boxes.data.cpu().numpy()
        objects = result[result[:,-1] == 0]

        # object 인식하면 1로 인식 못하면 0으로 topic 발행
        if len(objects) > 0:
            # tmp = np.reshape(objects, -1) #1차원 array로 만들기
            msg = Int32MultiArray()
            # tmp = tmp.astype(int).tolist()
            # msg.data = tmp
            msg.data = [1]
            self.detect_publisher.publish(msg) #/object_detect으로 퍼블리시
        else:
            msg = Int32MultiArray()
            msg.data = [0]
            self.detect_publisher.publish(msg)
        
        id = 0
        for row in objects:
            id += 1
            x1, y1, x2, y2, conf, cls = row
            cx = int(x1 + x2) // 2
            cy = int(y1 + y2) // 2
            center = (cx, cy)

            cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0,255, 0), 1)
            cv2.circle(color_image, center, 3, (0, 0, 255), -1)
            cv2.putText(color_image, "object " + str(id), (int(x1), int(y1)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("image", color_image)
        cv2.imshow("image", cv2.resize(color_image, (800, 600)))
        cv2.waitKey(1)


def yolo_main(args=None):
    rclpy.init(args=args)
    node = ObjectDetect()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    yolo_main()
