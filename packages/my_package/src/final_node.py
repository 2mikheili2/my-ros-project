#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped


# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 0     # angular vel   , rad/s     , counter clock wise (+)


class Solution(DTROS):
    def __init__(self, node_name):
        super(Solution, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # self._node_wheel_encoder = wheel_encoder_reader_node("wheel_encoder_node")
        self._node_wheel_control = WheelControlNode("wheel_control_node")
        # self._node_camera = camera_reader_node.("camera_reader_node")
        self._node_twist = TwistControlNode("twist_control_node")


class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            print(self._v)
            if self._v > 0.05:
                self._v -= 0.01
            # message = Twist2DStamped(v=self._v, omega=self._omega)
            # self._publisher.publish(message)
            self.pub()
            rate.sleep()
    
    def pub(self):
        message = Twist2DStamped(v=self._v, omega=self._omega)
        self._publisher.publish(message)

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

    def set_v(self, v):
        self._v = v

    def set_w(self, w):
        self._w = w

    def get_v(self):
        return self._v
    
    def get_w(self):
        return self._w

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
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

    def callback(self, msg):
        # convert JPEG bytes to CV image
        self._image = self._bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow(self._window, self._image)
        cv2.waitKey(1)

    def get_image(self):
        return self._image



# throttle and direction for each wheel
THROTTLE_LEFT = 0.5        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.5       # 30% throttle
DIRECTION_RIGHT = 1       # backward


class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        # form the message
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
            self._publisher.publish(message)
            rate.sleep()

    def pub(self):
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        self._publisher.publish(message)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

    def set_vel_left(self, vel_left):
        self._vel_left = vel_left

    def set_vel_right(self, vel_right):
        self._vel_right = vel_right



if __name__ == '__main__':    
    # keep spinning
    rospy.init_node('dual_dros_node', anonymous=False)
    sol = Solution("test_node")
    sol.run()
    rospy.spin()
