#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool

class RedLineDetectionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RedLineDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "redline-detection"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._image = None
        self._throtle = 0.5
        self.throtle_pub = rospy.Publisher('redline-throtle', Float64, queue_size = 1)
        self._init_x = None
        self._init_y = None
        self._x = None
        self._y = None 
        self._dist = None
        self._init_dist = None
        self._ref_vel = 1
        # self.red_pub = rospy.Publisher("red-state", Bool, queue_size = 1)
        # self.twist_pub = rospy.Publisher("velocity", Float64, queue_size = 1)
        self._state = True
        self._timer = None
        self._Kp = 1
        self._Ki = 0.01
        self._Kd = 0.0
        self._integral = 0
        self._derivative = 0
        self._vel = 0
        self._error = None

        self._lower_red1 = np.array([0, 120, 70])
        self._upper_red1 = np.array([10, 255, 255])
        self._lower_red2 = np.array([170, 120, 70])
        self._upper_red2 = np.array([180, 255, 255])

        self._h = 480
        self._w = 640

        self._mask_new = np.ones((self._h, self._w), dtype = np.uint8)
        self._scale = 1        

        self.left_vel = rospy.Publisher("left-angle", Float64, queue_size = 1)
        self.right_vel = rospy.Publisher("right-angle", Float64, queue_size = 1)

    def run(self):
    # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # print(self._state)
            # print(self._y, self._init_y)
            if self._state == True:
                self.redline()
                self.PID()
                print(self._vel)
                self.throtle_pub.publish(self._vel / 2)
            # self.red_pub.publish(self._state)
            rate.sleep()
        
    def update_state(self, data):
        self._state = data.data

    def callback(self, msg):
        # convert JPEG bytes to CV image
        self._image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow(self._window, self._image)
        # cv2.waitKey(1)       

    def redline(self):
        self.redline_detection()
        if self._init_y is not None:
            self._ref_vel = self._init_y / self._y
            # self._ref_vel -= (1 - self._ref_vel) * 0.05
            if self._ref_vel < 0.5:
                self._ref_vel = 0
            else:
                self._scale = 1 / self._ref_vel
                # self._init_dist = None
                # self._state = False
            # print(self._ref_vel)
            self._state = True

    def PID(self):
        t = rospy.get_time()
        d_t = 0.1
        if self._timer is not None:
            d_t = t - self._timer
        if d_t < 0.01:
            d_t = 0.01
        
        # print(d_t)
        self._timer = t
        error = self._ref_vel - self._vel

        self._integral += error * d_t
        if self._error is None:
            self._derivative = 0
        else:
            self._derivative = (error - self._error) / d_t
        self._error = error

        self._vel += self._Kp * self._error + self._Ki * self._integral + self._Kd * self._derivative 


    def redline_detection(self):
        
        image = self._image
        if image is None:
            return 
        image = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self._lower_red1, self._upper_red1)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        mask[ : int(2 * self._h // 5 * self._scale), : ] = 0
        mask[ :, : self._w // 2] = 0

        sobel_x = cv2.Sobel(mask, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3)
        
        sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))
        threshold_value = 150
        _, sobel_magnitude = cv2.threshold(sobel_magnitude, threshold_value, 255, cv2.THRESH_BINARY)

        coordinates = np.column_stack(np.where(sobel_magnitude == 255))
        avg_x = (np.max(coordinates[:, 1]))
        avg_y = (np.max(coordinates[:, 0]))
        if not np.isnan(avg_x) and not np.isnan(avg_y):
             avg_x = int(avg_x)
             avg_y = int(avg_y)
             self.update(avg_x, avg_y)
             sobel_magnitude[avg_y - 5 : avg_y + 5, avg_x - 5 : avg_x + 5] = 255
             # print(avg_x, avg_y)

        # cv2.imshow('Detected Red Rectangles', sobel_magnitude)
        # cv2.waitKey(1)

    # def on_shutdown(self):
    #     self._vel = 0
    #     self.twist_pub.publish(0)

    def update(self, x, y):
        if self._init_x is None:
            self._init_x = x
        if self._init_y is None:
            self._init_y = y
        self._x = x
        self._y = y

if __name__ == '__main__':
    # create the node
    node = RedLineDetectionNode(node_name='red_line_detection_node')
    # keep spinning
    node.run()
    rospy.spin()