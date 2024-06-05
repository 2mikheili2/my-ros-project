#!/usr/bin/env python3

import os
import rospy
from wheel_encoder_reader_node import WheelEncoderReaderNode
import wheel_encoder_reader_node
# from wheel_control_node import WheelControlNode 
# import wheel_control_node
# from camera_reader_node import CameraReaderNode
# import camera_reader_node 
from twist_control_node import TwistControlNode
import twist_control_node 

import cv2 
import numpy as np

from duckietown.dtros import DTROS, NodeType

class Solution():
    def __init__(self, node_name):
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._node_wheel_encoder = wheel_encoder_reader_node.node
        # self._node_wheel_control = wheel_control_node.node
        # self._node_camera = camera_reader_node.node
        self._node_twist = twist_control_node.node
        self._v = None
        self._w = None
        # self._node_wheel_control.set_vel_left(123)

    def r(self):
        # self._node_wheel_encoder.run()
        # self._node_wheel_control.run()
        # self._node_twist.run()
        pass 
    
    def run(self):
        self._node_twist.run()
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # self._node_wheel_control.pub()
            # self._node_twist.pub()
            # cv2.imshow("camera-reader", self._node_camera.get_image())
            # self.show_image()
            # image = self._node_camera.get_image()
            v = self._node_twist.get_v()
            # print(123)
            # WheelEncoderReaderNode.mess()
            print(v)
            if (v > 0.05):
                self._node_twist.set_v(v - 0.01)
            
            #
            # self.show_tick_left()
            # self.show_tick_right()
            rate.sleep()

    def show_image(self):
        image = self._node_camera.get_image()
        if image is None:
            return 
    #    def mid_value(x, y):
    #         return (x + y) / 2  
    #    # Convert the image to HSV color space
    #     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #     # Define the range for the red color in HSV space
    #     lower_red1 = np.array([0, 120, 70])
    #     upper_red1 = np.array([10, 255, 255])
    #     lower_red2 = np.array([170, 120, 70])
    #     upper_red2 = np.array([180, 255, 255])

    #     # Create masks for the red color
    #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    #     mask = cv2.bitwise_or(mask1, mask2)

    #     # Find contours in the mask
    #     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #     for contour in contours:
    #         # Approximate the contour to a polygon
    #         approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

    #         # Check if the approximated contour has 4 sides (is a rectangle)
    #         if len(approx) == 4:
    #             # Draw the rectangle on the original image
    #             first_diag_y = mid_value(approx[1][0][0], approx[3][0][0])
    #             second_diag_y = mid_value(approx[0][0][0], approx[2][0][0])
    #             center_y = mid_value(first_diag_y, second_diag_y)
    #             first_diag_x = mid_value(approx[1][0][1], approx[3][0][1])
    #             second_diag_x = mid_value(approx[0][0][1], approx[2][0][1])
    #             center_x = mid_value(first_diag_x, second_diag_x)
                
    #             cv2.drawContours(image, [approx], 0, (255, 0, 0), 5)
    #             image[int(center_x) - 5 : int(center_x) + 5  , int(center_y) - 5 :  int(center_y) + 5] = np.array([0, 0, 0])
    #             print(approx, center_x, center_y)

    #     # Show the final image with detected rectangles
        
        cv2.imshow("bla", image)
        cv2.waitKey(1)

    # def show_tick_left(self):
    #     left_tick = self._node_wheel_encoder.get_left_tick()
    #     print(left_tick)
    
    # def show_tick_right(self):
    #     right_tick = self._node_wheel_encoder.get_right_tick()
    #     print(right_tick)
    

if __name__ == '__main__':    
    # keep spinning
    sol = Solution("test_node")
    sol.run()
    rospy.spin()


# #!/usr/bin/env python3

# import os
# import rospy
# from duckietown.dtros import DTROS, NodeType
# from duckietown_msgs.msg import WheelsCmdStamped
# from std_msgs.msg import Float64

# # throttle and direction for each wheel
# THROTTLE_LEFT = 0.5        # 50% throttle
# DIRECTION_LEFT = 1         # forward
# THROTTLE_RIGHT = 0.5       # 30% throttle
# DIRECTION_RIGHT = 1       # backward


# class WheelControlNode(DTROS):

#     def __init__(self, node_name):
#         # initialize the DTROS parent class
#         super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         # static parameters
#         vehicle_name = os.environ['VEHICLE_NAME']
#         wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
#         # form the message
#         self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
#         self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
#         # construct publisher
#         self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
#         self.lane_follower_left_sub = rospy.Subscriber("lane_follower_left", Float64, self.set_left) 
#         self.lane_follower_right_sub = rospy.Subscriber("lane_follower_right", Float64, self.set_right) 
#         self.throtle_sub = rospy.Subscriber("redline_throtle", Float64, self.set_throtle)
    
#     def run(self):
#         # publish 10 messages every second (10 Hz)
#         rate = rospy.Rate(1)
#         while not rospy.is_shutdown():
#             self.pub()
#             rate.sleep()

#     def set_left(self, vel):
#         if vel.data is not None:
#             DIRECTION_LEFT = vel.data
        
#     def set_right(self, vel):
#         if vel.data is not None:
#             DIRECTION_RIGHT = vel.data
    
#     def set_throtle(self, thro):
#         if thro.data is not None:
#             THROTTLE_LEFT = thro.data
#             THROTTLE_RIGHT = thro.data 

#     def pub(self):
#         print(self._vel_left, self._vel_right) 
#         self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
#         self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
#         message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
#         self._publisher.publish(message)

#     def on_shutdown(self):
#         stop = WheelsCmdStamped(vel_left=0, vel_right=0)
#         self._publisher.publish(stop)

# if __name__ == '__main__':
#     # create the node
#     node = WheelControlNode(node_name='wheel_control_node')
#     # run node
#     node.run()
#     # keep the process from terminating
#     rospy.spin()
