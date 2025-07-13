#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class OrangeDetector(Node):
    def __init__(self):
        super().__init__('orange_detector')
        self.sub = self.create_subscription(Image, '/front_camera/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.alert_triggered = False  

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([10, 100, 100])
        upper = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        area = cv2.countNonZero(mask)

        if area > 50000 and not self.alert_triggered:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)  
                cv2.putText(img, 'Orange Cone', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            self.get_logger().warn("🟠 Orange Cone Detected VERY CLOSE!")
            self.alert_triggered = True
        else:
            self.alert_triggered = False  

        cv2.imshow("Detection", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = OrangeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

