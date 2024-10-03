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

        # Field dimensions and zone limits
        self.field_length = 2.0  # meters
        self.field_width = 2.0  # meters
        self.half_field = self.field_length / 2.0
        self.penalty_zone_size = 0.2  # meters
        self.goal_area_width = 0.8  # meters
        self.goal_area_height = 0.2  # meters
        self.center_x = 0
        self.center_y = 0
        self.Restriction = False
        # Load YOLOv8 model
        try:
            self.model = YOLO("/home/areej/catkin_ws/src/finalComp_Group2/models/best (4).pt")
        except Exception as e:
            rospy.logerr("Failed to load YOLOv8 model: %s", e)
            rospy.signal_shutdown("Model load failure")

        # Subscribe to depth and RGB image topics
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)

        # To store depth and RGB images
        self.depth_image = None
        self.rgb_image = None

        rospy.loginfo("YOLOv8BallChaser node initialized.")
        rospy.spin()

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
                # Print bounding boxes, class IDs, and class names
                boxes = result.boxes.xyxy.numpy()  # Convert to numpy array
                class_ids = result.boxes.cls.numpy()  # Convert to numpy array
                names = result.names
                
                for box, class_id in zip(boxes, class_ids):
                    rospy.loginfo("Detected box: %s, Class ID: %d", box, class_id)
                    if names[int(class_id)] == 'balls':
                        x1, y1, x2, y2 = map(int, box)
                        self.center_x = int((x1 + x2) / 2)
                        self.center_y = int((y1 + y2) / 2)
                        rospy.loginfo("First if ")

                        if 0 <= self.center_x < self.depth_image.shape[1] and 0 <= self.center_y < self.depth_image.shape[0]:
                            target_depth = self.depth_image[self.center_y, self.center_x]
                            rospy.loginfo("Second if ")

                            if not np.isnan(target_depth) and target_depth > 0:
                                rospy.loginfo("Target depth at center (%d, %d): %f", self.center_x, self.center_y, target_depth)

                            else:
                                rospy.logwarn("Invalid depth value detected")
                        else:
                            rospy.logwarn("Bounding box center out of bounds: (%d, %d)", self.center_x, self.center_y)
        except Exception as e:
            rospy.logwarn("Detection or movement failed: %s", e)


    def move_to_ball(self, target_depth):
        RightLeft= 1
        rospy.loginfo(f"Moving to ball with target depth: {target_depth}")
        if target_depth is None:
            return
        
        image_center_x = self.depth_image.shape[1] / 2
        image_center_y = self.depth_image.shape[0] / 2

        # Define sight margins
        sight_margin_x = 10  # Horizontal margin around the image center
        sight_margin_y = 10  # Vertical margin around the image center

        # Check if the ball is in the sight area
        ball_in_sight_x = abs(self.center_x - image_center_x) < sight_margin_x
        ball_in_sight_y = abs(self.center_y - image_center_y) < sight_margin_y
        ball_in_sight = ball_in_sight_x and ball_in_sight_y
        self.vel_msg.linear.x = 0.0
        rospy.sleep(0.1)
        if not ball_in_sight:
            rospy.logwarn("Ball is not in sight. Performing specific logic.")
            if RightLeft == 1 : #move right 
                self.vel_msg.linear.y = -0.5
                rospy.sleep(0.8)
                self.vel_msg.linear.y = 0.0
                rospy.sleep(0.1)
                if 0.2 < target_depth <= 1.0:
                    self.vel_msg.linear.x = 0.8
                    rospy.sleep(0.4)
                    self.vel_msg.linear.x = 0.0
                    rospy.sleep(0.1)
                    self.vel_msg.linear.x = -0.8
                    rospy.sleep(0.3)
                    self.vel_msg.linear.x = 0.0
                    rospy.sleep(0.1)
                    self.vel_msg.linear.y = 0.5
                    rospy.sleep(0.8)
                    self.vel_msg.linear.y = 0.0
                    rospy.sleep(0.1)
                    RightLeft = 2
            if RightLeft == 2 : # move left
                self.vel_msg.linear.y = 0.5
                rospy.sleep(0.8)
                self.vel_msg.linear.y = 0.0
                rospy.sleep(0.1)
                if 0.2 < target_depth <= 1.0:
                    self.vel_msg.linear.x = 0.8
                    rospy.sleep(0.4)
                    self.vel_msg.linear.x = 0.0
                    rospy.sleep(0.1)
                    self.vel_msg.linear.x = -0.8
                    rospy.sleep(0.3)
                    self.vel_msg.linear.x = 0.0
                    rospy.sleep(0.1)
                    self.vel_msg.linear.y = -0.5
                    rospy.sleep(0.8)
                    self.vel_msg.linear.y = 0.0
                    rospy.sleep(0.1)
                    RightLeft = 1
                    
            return

        else :
                if 0.2 < target_depth <= 1.0:
                    self.vel_msg.linear.x = 0.8
                    rospy.sleep(0.4)
                    self.vel_msg.linear.x = 0.0
                    rospy.sleep(0.1)
                    self.vel_msg.linear.x = -0.8
                    rospy.sleep(0.3)
                    self.vel_msg.linear.x = 0.0
                    rospy.sleep(0.1)

        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        YOLOv8BallChaser()
    except rospy.ROSInterruptException:
        pass
