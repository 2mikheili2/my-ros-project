#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridges
from std_msgs.msg import Bool
import cv2
import numpy as np

class NewNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(NewNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._image = None
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        VELOCITY = 0.0
        OMEGA = 0.0
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        # self._remote = rospy.Publisher('camera_detected', Bool, queue_size = 1)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        self._image = self._bridge.compressed_imgmsg_to_cv2(msg)
        self._image[ : self._image.shape[0] // 2, : ] = np.array([0, 0, 0])
        self._image = self.rectangle_detection()
        cv2.imshow(self._window, self._image)
        self._remote.publish(self._image)
        # self._sub = rospy.Subscriber('camera_detected', Bool, callback, queue_size = 1)
        cv2.waitKey(1)


    def rectangle_detection(self):
        image = self._image
        if image is None:
            return 
        def mid_value(x, y):
            return (x + y) / 2  
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range for the red color in HSV space
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Create masks for the red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Approximate the contour to a polygon
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

            # Check if the approximated contour has 4 sides (is a rectangle)
            if len(approx) == 4:
                # Draw the rectangle on the original image
                first_diag_y = mid_value(approx[1][0][0], approx[3][0][0])
                second_diag_y = mid_value(approx[0][0][0], approx[2][0][0])
                center_y = mid_value(first_diag_y, second_diag_y)
                first_diag_x = mid_value(approx[1][0][1], approx[3][0][1])
                second_diag_x = mid_value(approx[0][0][1], approx[2][0][1])
                center_x = mid_value(first_diag_x, second_diag_x)
                len_y = mid_value(abs(approx[1][0][0] - approx[0][0][0]), 
                                  abs(approx[2][0][0] - approx[3][0][0]))
                len_x = mid_value(abs(approx[1][0][1] - approx[0][0][1]), 
                                  abs(approx[2][0][1] - approx[3][0][1])) 
                if len_x > 5 and len_y > 5:
                    cv2.drawContours(image, [approx], 0, (255, 0, 0), 5)
                    image[int(center_x) - 5 : int(center_x) + 5  , int(center_y) - 5 :  int(center_y) + 5] = np.array([0, 0, 0])
        return image

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                # start printing values when received from both encoders
                msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
                # rospy.loginfo(msg)  
                # self.dec_vel()
                message = Twist2DStamped(v=self._v, omega=self._omega)
                self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

    def dec_vel(self):
        if self._v > 0.05:
            self._v -= 0.01
if __name__ == '__main__':    
    # keep spinning
    sol = NewNode("new_node")
    sol.run()
    rospy.spin()
