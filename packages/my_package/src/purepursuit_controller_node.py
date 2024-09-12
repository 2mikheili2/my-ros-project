#!/usr/bin/env python3
import threading
import time
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
from rospy.numpy_msg import numpy_msg


def nothing(x):
    pass


class LaneFollowerNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowerNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        # construct subscriber
        self.sub2 = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.redline)
        self.sub = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(
            wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_motor = rospy.Publisher("lane-follower-left", Float64, queue_size=1)
        self.right_motor = rospy.Publisher(
            "lane-follower-right", Float64, queue_size=1)

        self.left = 0.2
        self.right = 0.2
        self.gain = 0.2
        self.const = 0.25

        self._ref_vel = 1

        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

        self.red_line_detected = False
        self.has_stopped = False
        self.red_image = None

        self._state = True
        self._timer = None
        self._Kp = 1
        self._Ki = 0.01
        self._Kd = 0.0
        self._integral = 0
        self._derivative = 0
        self._vel = 1
        self._error = None

        def stop_and_turn():
            # Stop for 1 second
            self.left_motor.publish(0)
            self.right_motor.publish(0)
            rospy.sleep(1)

            # Move straight for 5 seconds
            self.left_motor.publish(0.5)
            self.right_motor.publish(0.5)
            rospy.sleep(5)

            # Turn right
            self.left_motor.publish(0.5)
            self.right_motor.publish(0)
            rospy.sleep(0.2)

            # Reset to normal operation
            self.red_line_detected = False

        thread = threading.Thread(target=stop_and_turn)
        thread.start()

    def shutdown_hook(self):
        self.shutting_down = True
        self.left_motor.publish(0)
        self.right_motor.publish(0)
        cv2.destroyAllWindows()  # Close the OpenCV window

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


    def redline(self, msg):
        # self.PID()
        self.has_stopped = not self.has_stopped
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # image = cv2.bilateralFilter(image, 12, 125, 155)
        yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
        lb_red = np.array([0, 0, 170])
        ub_red = np.array([2172, 5204, 10000])
        mask_red = cv2.inRange(yuv, lb_red, ub_red)
        red_image = cv2.bitwise_and(image, image, mask=mask_red)
        red_image[:300, :] = 0

        gray = cv2.cvtColor(red_image, cv2.COLOR_BGR2GRAY)

        # Find contours in the image
        contours, _ = cv2.findContours(
            gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        area = 0
        # Filter horizontal blocks and print "found" when detected
        x, y, w, h, aspect_ratio = 0, 0, 0, 0, 0
        for cnt in contours:
            max_contour = max(contours, key=cv2.contourArea)

            # Get bounding box and calculate aspect ratio
            x, y, w, h = cv2.boundingRect(max_contour)

            aspect_ratio = float(w)/h  # width/height

            area = max(area, w*h)
            # if aspect_ratio >= 2:  # Adjust this value to allow some deviation
                # print("Found")
                # cv2.rectangle(red_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        self.red_image = red_image
        # print("callback2")
        red_lower_edge = y + h

        if aspect_ratio > 2:
            self._ref_vel = 1 - red_lower_edge / \
                480 if self._ref_vel >= 0.02 else self._ref_vel
        else:
            self._ref_vel = 1
        if area > 40000 and not self.has_stopped and not self.red_line_detected:
            self.has_stopped = True
            self.left_motor.publish(0)
            self.right_motor.publish(0)
            rospy.sleep(1)
            self.left_motor.publish(0.4)
            self.right_motor.publish(0.1)
            rospy.sleep(2.3)
            self.has_stopped = False
            self.red_line_detected = True
        print(abs(self._ref_vel - self._vel))

    def callback(self, msg):    
    
        if self.shutting_down or self.has_stopped:
            return

        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # GOOD VALUES
        # self.image = cv2.bilateralFilter(self.image, 12, 125, 155)

        # display updated frame with correcgt hsv values

        luv = cv2.cvtColor(self.image, cv2.COLOR_BGR2LUV)
        yuv = cv2.cvtColor(self.image, cv2.COLOR_BGR2YUV)
        hls = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)

        lb_yellow = np.array([0, 0, 170])
        ub_yellow = np.array([2172, 5204, 10000])
        mask_yellow = cv2.inRange(luv, lb_yellow, ub_yellow)
        yellow_image = cv2.bitwise_and(
            self.image, self.image, mask=mask_yellow)
        yellow_image[:240, :] = 0
        kernel = np.ones((7, 7), np.uint8)
        yellow_image = cv2.dilate(yellow_image, kernel, iterations=2)

        # sobel_x = cv2.Sobel(mask_white, cv2.CV_64F, 1, 0, ksize=3)
        # sobel_y = cv2.Sobel(mask_white, cv2.CV_64F, 0, 1, ksize=3)
        # sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        # sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))

        lb_white = np.array([0, 173, 0])
        ub_white = np.array([179, 255, 255])
        mask_white = cv2.inRange(hls, lb_white, ub_white)
        white_image = cv2.bitwise_and(self.image, self.image, mask=mask_white)
        white_image[:200, :] = 0
        white_image[:, :200] = 0

        # NORMAL VALUES

        lb_red = np.array([0, 0, 170])
        ub_red = np.array([2172, 5204, 10000])
        mask_red = cv2.inRange(yuv, lb_red, ub_red)
        red_image = cv2.bitwise_and(self.image, self.image, mask=mask_red)
        red_image[:300, :] = 0

        tmp = cv2.bitwise_or(yellow_image, self.red_image)
        combined_img = cv2.bitwise_or(tmp, white_image)

        white_color_count = np.count_nonzero(white_image)
        yellow_color_count = np.count_nonzero(yellow_image)

        white_color_count = white_color_count / (640*480)
        yellow_color_count = yellow_color_count / (640*480)

        left_motor = (self.const + self.gain * (self.left *
                      white_color_count)) * self._ref_vel
        right_motor = (self.const + self.gain * (self.right *
                       yellow_color_count)) * self._ref_vel

        if white_color_count < 0.1 and yellow_color_count < 0.1:
            self.left_motor.publish(-0.8)
            self.right_motor.publish(-0.8)
        elif abs(white_color_count - yellow_color_count) < 0.05:
            self.left_motor.publish(0.3)
            self.right_motor.publish(0.3)
        if white_color_count >= yellow_color_count:
            left_motor = left_motor - 0.2 if left_motor > 0.2 else left_motor
            right_motor += 0.25
            if not self.shutting_down:
                self.left_motor.publish(left_motor)
                self.right_motor.publish(right_motor)

        elif yellow_color_count > white_color_count:
            right_motor = right_motor - 0.2 if right_motor > 0.2 else right_motor
            left_motor += 0.25
            if not self.shutting_down:
                self.left_motor.publish(left_motor)
                self.right_motor.publish(right_motor)

        combined_img[:, ::20] = [0, 0, 255]
        combined_img[::20, :] = [0, 0, 255]
        combined_img[:, 320] = [255, 0, 0]
        combined_img[240, :] = [255, 0, 0]

        # cv2.imshow(self._window, combined_img)
        # cv2.waitKey(1)


if __name__ == '__main__':
    # create the node
    node = LaneFollowerNode(node_name='lane_follower_node')
    # keep spinning
    rospy.spin()