#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool
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
        try:
            self.model = YOLO("/home/areej/catkin_ws/src/mia_competition/models/best (4).pt")
        except Exception as e:
            rospy.logerr("Failed to load YOLOv8 model: %s", e)
            rospy.signal_shutdown("Model load failure")

        # Subscribe to depth and RGB image topics
        self.restriction = rospy.Subscriber('/Restriction', Bool, restriction_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)

        # To store depth and RGB images
        self.depth_image = None
        self.rgb_image = None
        self.restriction = True

        rospy.loginfo("YOLOv8BallChaser node initialized.")
        rospy.spin()
    def restriction_callback(self,msg:bool):
        restriction = msg
    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            rospy.loginfo("Depth image received with shape: %s", str(self.depth_image.shape))

            if self.rgb_image is not None:
                self.detect_and_move(self.rgb_image)
        except Exception as e:
            rospy.logwarn("Failed to process depth image: %s", e)

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("RGB image received with shape: %s", str(self.rgb_image.shape))

            if self.depth_image is not None:
                self.detect_and_move(self.rgb_image)
        except Exception as e:
            rospy.logwarn("Failed to process RGB image: %s", e)

    def detect_and_move(self, rgb_image):
        if self.depth_image is None or rgb_image is None:
            rospy.logwarn("Missing depth or RGB image.")
            return

        try:
            results = self.model(rgb_image)
            rospy.loginfo("YOLO detection results: %s", results)

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy()
                names = result.names
                
                for box, class_id in zip(boxes, class_ids):
                    rospy.loginfo("Detected box: %s, Class ID: %d", box, class_id)
                    if names[int(class_id)] == 'balls':
                        x1, y1, x2, y2 = map(int, box)
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        rospy.loginfo("first if ")

                        if 0 <= center_x < self.depth_image.shape[1] and 0 <= center_y < self.depth_image.shape[0]:
                            target_depth = self.depth_image[center_y, center_x]
                            rospy.loginfo("2 if ")


                            if not np.isnan(target_depth) and target_depth > 0:
                                rospy.loginfo("Target depth at center (%d, %d): %f", center_x, center_y, target_depth)
                                self.move_to_ball(target_depth)
                                rospy.loginfo("3 if ")

                            else:
                                rospy.logwarn("Invalid depth value detected")
                        else:
                            rospy.logwarn("Bounding box center out of bounds: (%d, %d)", center_x, center_y)
        except Exception as e:
            rospy.logwarn("Detection or movement failed: %s", e)

    def move_to_ball(self, target_depth):
        rospy.loginfo(f"Moving to ball with target depth: {target_depth}")
        if restriction == False :
            if target_depth is None:
                return

            if target_depth > 1.0:
                self.vel_msg.linear.x = 0.5
            elif 0.2 < target_depth <= 1.0:
                self.vel_msg.linear.x = 1.0
            elif target_depth <= 0.2:
                self.vel_msg.linear.x = 2.0  # Shoot

            self.velocity_publisher.publish(self.vel_msg)
        else: 
            
if __name__ == '__main__':
    try:
        YOLOv8BallChaser()
    except rospy.ROSInterruptException:
        pass
