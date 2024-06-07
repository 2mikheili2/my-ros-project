

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
        # self.twist_pub = rospy.Publisher("velocity", Float64, queue_size = 10)
        self._state = True
        self._timer = None
        self._Kp = 1
        self._Ki = 0.01
        self._Kd = 0.01
        self._integral = 0
        self._derivative = 0
        self._vel = 0
        self._error = None

    def run(self):
    # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # self.throtle_pub.publish(self._slow)
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
        
            # self._state = False
        # if self._init_dist is None:
        #     print("Yes")
        # else:
        #     print("No")               

    def redline(self):
        self.redline_detection()
        if self._init_y is not None:
            self._ref_vel = self._init_y / self._y
            # self._ref_vel -= (1 - self._ref_vel) * 0.05
            if self._ref_vel < 0.5:
                self._ref_vel = 0
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
        # Convert the image to HSV color space
        image = self._image
        if image is None:
            return None
        image = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Define the range for the red color in HSV space
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        # Create masks for the red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask_new = np.ones(mask.shape, dtype = np.uint8)
        mask_new[ : int(2 * mask.shape[0] // 5), : ] = 0
        mask_new[ :, : mask.shape[1] // 2] = 0
        sobel_x = cv2.Sobel(mask, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3)
        # Compute the magnitude of the gradient
        sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))
        threshold_value = 150
        _, sobel_magnitude = cv2.threshold(sobel_magnitude, threshold_value, 255, cv2.THRESH_BINARY)
        # print(np.sum(sobel_magnitude) / 255, sobel_magnitude.shape[0] * sobel_magnitude.shape[1])
        sobel_magnitude = sobel_magnitude * mask_new
        coordinates = np.column_stack(np.where(sobel_magnitude == 255))
        # Calculate average x and y coordinates
        avg_x = (np.max(coordinates[:, 1]))
        avg_y = (np.max(coordinates[:, 0]))
        if not np.isnan(avg_x) and not np.isnan(avg_y):
             avg_x = int(avg_x)
             avg_y = int(avg_y)
             self.update(avg_x, avg_y)
             # print(self._init_y, self._y)
             sobel_magnitude[avg_y - 5 : avg_y + 5, avg_x - 5 : avg_x + 5] = 255
             # print(avg_x, avg_y)
        # Show the final image with detected rectangles
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
        # if self._init_dist is None:
        #     self._init_dist = np.hypot(self._init_x, self._init_y)
        self._x = x
        self._y = y
        # self._dist = np.hypot(self._x, self._y)

if __name__ == '__main__':
    # create the node
    node = RedLineDetectionNode(node_name='red_line_detection_node')
    # keep spinning
    node.run()
    rospy.spin()