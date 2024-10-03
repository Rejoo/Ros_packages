#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YOLOv8BallChaser:
    def __init__(self):
        rospy.init_node('yolov8_ball_chaser')

        self.bridge = CvBridge()
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()

        # Load YOLOv8 model
        self.model = YOLO("/home/areej/catkin_ws/src/mia_competition/models/best (4).pt")  # Path to your YOLOv8 model (e.g., "yolov8n.pt")

        # Subscribe to depth and RGB image topics
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)

        # To store depth and RGB images
        self.depth_image = None
        self.rgb_image = None

        rospy.spin()

    def depth_callback(self, data):
        # Convert ROS depth Image message to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        rospy.loginfo("Depth image received with shape: %s", str(self.depth_image.shape))

        # If both images are available, detect objects and move
        if self.rgb_image is not None:
            self.detect_and_move(self.rgb_image)

    def rgb_callback(self, data):
        # Convert ROS RGB Image message to OpenCV format
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        rospy.loginfo("RGB image received with shape: %s", str(self.rgb_image.shape))

        # If both images are available, detect objects and move
        if self.depth_image is not None:
            self.detect_and_move(self.rgb_image)

    def detect_and_move(self, rgb_image):
        if self.depth_image is None or rgb_image is None:
            rospy.logwarn("Missing depth or RGB image.")
            return
        
        # Run YOLOv8 inference
        results = self.model(rgb_image)
        rospy.loginfo("YOLO detection results: %s", results)
        
        for result in results:
            # Use the available attributes
            boxes = result.boxes.xyxy.numpy()  # Get bounding boxes as numpy array
            class_ids = result.boxes.cls.numpy()  # Get class IDs as numpy array
            names = result.names  # Get class names
            
            for box, class_id in zip(boxes, class_ids):
                if names[int(class_id)] == 'ball':  # Check for 'ball' class
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = map(int, box)

                    # Get bounding box center coordinates
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Retrieve depth at the center of the bounding box
                    target_depth = self.depth_image[center_y, center_x]

                    # Move robot based on the detected ball's depth
                    self.move_to_ball(target_depth)
                    rospy.loginfo(f"target depth 1 :{target_depth}")

    def move_to_ball(self, target_depth):
        rospy.loginfo("i am in move to ball")
        rospy.loginfo(f"target depth 2 :{target_depth}")
        if target_depth is None:
            return

        # Logic for moving based on depth
        if target_depth > 1.0:
            self.vel_msg.linear.x = 0.5
        elif 0.2 < target_depth <= 1.0:
            self.vel_msg.linear.x = 1.0
        elif target_depth <= 0.2:
            self.vel_msg.linear.x = 2.0  # Shoot

        # Publish the velocity command
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        YOLOv8BallChaser()
    except rospy.ROSInterruptException:
        pass
