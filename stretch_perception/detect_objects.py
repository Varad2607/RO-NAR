from ultralytics import YOLOWorld

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import numpy as np
import cv2


# Detect objects in camera feed and publish classes detected

class Detector(Node):
    def __init__(self):
        super().__init__('detect_objects')
        # create rgb subscriber
        self.rgb_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, 1)

        # create detection publisher
        self.detection_publisher = self.create_publisher(Float64MultiArray, '/fix/detections', 10)

        self.processor = self.create_timer(1, self.process_cb)

        self.rgb = None

        self.original_size = None
        self.modified_size = None

        # put classes here
        self.classes = ['clothes']

        # load yolo world
        self.detector = YOLOWorld('yolov8l-world.pt')
        self.detector.set_classes(self.classes)


    def rgb_cb(self, msg):
        self.rgb = msg


    def process_cb(self):
        if self.rgb is None:
            return
        
        # process image
        image = cv2.rotate(cv2.cvtColor(np.reshape(np.array(self.rgb.data), (self.rgb.height, self.rgb.width, 3)), 
            cv2.COLOR_RGB2BGR), # color should be in bgr format
            cv2.ROTATE_90_CLOCKWISE) # make sure image is in correct orientation
        
        if self.original_size is None or self.modified_size is None:
            self.original_size = image.shape[:2]
            self.modified_size = (self.original_size[0] - self.original_size[0] % 32, self.original_size[1] - self.original_size[1] % 32)
        
        results = self.detector.predict(image[:self.modified_size[0], :self.modified_size[1], :], imgsz=self.modified_size, conf=0.5)
        frame_classes = results[0].boxes.cls.tolist()

        detection_msg = Float64MultiArray()
        detection_msg.data = frame_classes

        # publish classes detected
        self.detection_publisher.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)

    detector = Detector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()