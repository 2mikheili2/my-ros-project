#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float64, Bool

# throttle and direction for each wheel
# THROTTLE_LEFT = 0.2        # 50% throttle
# DIRECTION_LEFT = 1         # forward
# THROTTLE_RIGHT = 0.2       # 30% throttle
# DIRECTION_RIGHT = 1       # backward


class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        # form the message
        self._left_throtle = 0.5
        self._right_throtle = 0.5
        self._left_direction = 0.5
        self._right_direction = 0.5
        self._vel_left = self._left_throtle * self._left_direction
        self._vel_right = self._right_throtle * self._right_direction
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.lane_follower_left_sub = rospy.Subscriber("lane-follower-left", Float64, self.set_left) 
        self.lane_follower_right_sub = rospy.Subscriber("lane-follower-right", Float64, self.set_right) 
        self.throtle_sub = rospy.Subscriber("redline-throtle", Float64, self.set_throtle)
        self.red_sub = rospy.Subscriber("red-state", Bool, self.update_state)
        self._state = False

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._state  == False:
                self.pub()
            rate.sleep()

    def update_state(self, vel):
        self._state = vel.data

    def set_left(self, vel):
        # print(vel.data)
        if vel.data is not None and vel.data > -1 and vel.data < 1:
            self._left_direction = vel.data
        
    def set_right(self, vel):
        if vel.data is not None and vel.data > -1 and vel.data < 1:
            self._right_direction = vel.data
    
    def set_throtle(self, thro):
        if thro.data is not None:
            self._left_throtle = thro.data
            self._right_throtle = thro.data 

    def pub(self):
        # print(self._vel_left, self._vel_right, self._right_throtle) 
        # print(self._left_throtle)
        self._vel_left = self._left_throtle * self._left_direction
        self._vel_right = self._right_throtle * self._right_direction
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        self._publisher.publish(message)

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
