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

                                # Ensure robot stays within allowed zones
                                if self.is_in_allowed_zone(self.center_x, self.center_y):
                                    self.move_to_ball(target_depth)
                                    rospy.loginfo("Third if ")
                                else:
                                    rospy.logwarn("Ball detected in restricted zone.")
                            else:
                                rospy.logwarn("Invalid depth value detected")
                        else:
                            rospy.logwarn("Bounding box center out of bounds: (%d, %d)", center_x, center_y)
        except Exception as e:
            rospy.logwarn("Detection or movement failed: %s", e)

    def is_in_allowed_zone(self, center_x, center_y):
        """
        Check if the ball is in the permitted zone.
        The robot is not allowed to enter the opponent's half, penalty zones, or goal area.
        """
        # Convert image coordinates to field coordinates
        field_x = center_x * self.field_length / self.depth_image.shape[1]
        field_y = center_y * self.field_width / self.depth_image.shape[0]

        # Check if the robot is in the opponent's half
        if field_x > self.half_field:
            rospy.logwarn("Ball detected in opponent's half")
            
            return False

        # Check if the robot is in a penalty zone (corners of the field)
        if (field_x < self.penalty_zone_size and field_y < self.penalty_zone_size) or \
           (field_x < self.penalty_zone_size and field_y > self.field_width - self.penalty_zone_size) or \
           (field_x > self.half_field - self.penalty_zone_size and field_y < self.penalty_zone_size) or \
           (field_x > self.half_field - self.penalty_zone_size and field_y > self.field_width - self.penalty_zone_size):
            rospy.logwarn("Ball detected in penalty zone")
            return False

        # Check if the robot is in the goal area
        if field_x < self.goal_area_height and \
           (self.field_width / 2 - self.goal_area_width / 2 <= field_y <= self.field_width / 2 + self.goal_area_width / 2):
            rospy.logwarn("Ball detected in goal area")
            return False

        return True
    


    def move_to_ball(self, target_depth):
        var = 0 
        rospy.loginfo(f"Moving to ball with target depth: {target_depth}")
        if target_depth is None:
            return
        
         # Calculate deviation of the ball from the center of the image
        image_center_x = self.depth_image.shape[1] / 2

        if  self.is_in_allowed_zone(self.center_x, self.center_y):
                # if target_depth > 1.0:
                #     self.vel_msg.linear.x = 0.5
                if var == 0: 
                    self.vel_msg.linear.x = 0.6
                    var = 1
                if 0.2 < target_depth <= 1.0:
                    self.vel_msg.linear.x = 0.6
                    rospy.sleep(0.2)
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.x = -0.6
                    rospy.sleep(0.2)
                    self.vel_msg.linear.x = 0.0

                # Move sideways based on ball's horizontal position
                if self.center_x < image_center_x - 30:  # Ball is on the left side of the image
                    self.vel_msg.linear.y = -0.3  # Move to the left
                    rospy.sleep(0.2)
                    self.vel_msg.linear.y = 0.0 
                    self.vel_msg.linear.x = 0.6
                    rospy.sleep(0.2)
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.x = -0.6
                    self.vel_msg.linear.y = 0.3
                    rospy.sleep(0.2)
                    self.vel_msg.linear.y = 0.0

                elif self.center_x > image_center_x + 30:  # Ball is on the right side of the image
                    self.vel_msg.linear.y = 0.3  # Move to the right
                    rospy.sleep(0.2)
                    self.vel_msg.linear.y = 0.0 
                    self.vel_msg.linear.x = 0.6
                    rospy.sleep(0.2)
                    self.vel_msg.linear.x = 0.0
                    self.vel_msg.linear.y = -0.6
                    self.vel_msg.linear.y = -0.3
                    rospy.sleep(0.2)
                    self.vel_msg.linear.y = 0.0

                else:
                    self.vel_msg.linear.y = 0.0  # No sideways movement if ball is centered   

                # # Ensure robot does not enter restricted zones
                # if not self.is_in_allowed_zone(self.center_x, self.center_y):
                #     self.vel_msg.linear.x = -0.6
                #     rospy.logwarn("Robot is entering a restricted zone, stopping.")

        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    try:
        YOLOv8BallChaser()
    except rospy.ROSInterruptException:
        pass
