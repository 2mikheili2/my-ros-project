#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool
import math
class LaneFollowerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "lane-follower"
        # cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._image = None
        self._left_velocity = None
        self._right_velocity = None
        self.l_max = -math.inf
        self.r_max = -math.inf
        self.l_min = math.inf
        self.r_min = math.inf
        self.left_pub = rospy.Publisher('lane-follower-left', Float64, queue_size = 1)
        self.right_pub = rospy.Publisher('lane-follower-right', Float64, queue_size = 1)
        self.red_sub = rospy.Subscriber("red-state", Bool, self.update_state)
        self._state = False

    def run(self):
    # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # print(self._state)
            if self._state == False:
                self.left_pub.publish(self._left_velocity)
                self.right_pub.publish(self._right_velocity)
            rate.sleep()

    def update_state(self, state):
        self._state = state.data

    def callback(self, msg):
        # convert JPEG bytes to CV image
        self._image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow(self._window, self._image)
        # cv2.waitKey(1)
        self._left_velocity = 2
        self._right_velocity = 2
        # print(self._left_velocity)
        image_left = self.left_yellow(self._image)
        image_right = self.right_white(self._image)
        l = float(np.sum(image_left))
        r = float(np.sum(image_right))
        self.change_velocity(l, r)
        # cv2.imshow('left', image_left)
        # cv2.waitKey(1)
        # cv2.imshow('right', image_right)
        # cv2.waitKey(1)
    
    def change_velocity(self, l, r):
        # l = min(0.95, max(l, 0.05))
        # r = min(0.95, max(r, 0.05)) 
        s = 2 * l + r
        if s == 0:
            l = 0.5
            r = 0.5
        else :
            l = (2 * l) / s
            r = r / s
        # l = max(l, 0.01)
        # r = max(r, 0.01)
        l = l
        r = r
        # if l == 0.02:
        #     r = 1.02
        # if r == 0.02:
        #     l = 1.02 
        # print(l, r)
        self._left_velocity = l
        self._right_velocity = r

    # Define the range for the red color in HSV space

    def right_white(self, image):
        mask_new = np.ones((image.shape[0], image.shape[1]), dtype = np.uint8)
        mask_new[: image.shape[0] // 2, : ] = 0
        width = image.shape[1]
        mask_left = np.ones(mask_new.shape)
        mask_left[:,int(np.floor(width/2)):width + 1] = 0
        mask_right = np.ones(mask_new.shape)
        mask_right[:,0:int(np.floor(width/2))] = 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 185])         # CHANGE ME
        upper_white = np.array([179, 70, 255])   # CHANGE ME
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_white = cv2.GaussianBlur(mask_white, (5, 5), 0)
        sobel_x = cv2.Sobel(mask_white, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(mask_white, cv2.CV_64F, 0, 1, ksize=3)
        sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))
        threshold_value = 47
        _, mask_mag = cv2.threshold(sobel_magnitude, threshold_value, 255, cv2.THRESH_BINARY)
        # mask_sobelx_pos = (sobel_x > 0)
        # mask_sobely_neg = (sobel_y < 0)
        mask_right_edge = mask_new * mask_right * mask_mag  #  * mask_sobelx_pos * mask_sobely_neg
        return mask_right_edge
    
    def left_yellow(self, image):
        mask_new = np.ones((image.shape[0], image.shape[1]), dtype = np.uint8)
        mask_new[: image.shape[0] // 2, : ] = 0
        width = image.shape[1]
        mask_left = np.ones(mask_new.shape)
        mask_left[:,int(np.floor(width/2)):width + 1] = 0
        mask_right = np.ones(mask_new.shape)
        mask_right[:,0:int(np.floor(width/2))] = 0

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([9, 129, 155])
        upper_yellow = np.array([83, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        sobel_x = cv2.Sobel(mask_yellow, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(mask_yellow, cv2.CV_64F, 0, 1, ksize=3)
        sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))
        threshold_value = 27
        _, mask_mag = cv2.threshold(sobel_magnitude, threshold_value, 255, cv2.THRESH_BINARY)
        # mask_sobelx_neg = (sobel_x < 0)
        # mask_sobely_neg = (sobel_y < 0)
        mask_left_edge = mask_new * mask_left * mask_mag # * mask_sobelx_neg * mask_sobely_neg
        return mask_left_edge
    


if __name__ == '__main__':
    # create the node
    node = LaneFollowerNode(node_name='lane_follower_node')
    # keep spinning
    node.run()
    rospy.spin()