#!/usr/bin/env python3
import os
import threading
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Bool

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
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.lanefollower)
        self.sub_red_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.redline)
        self._image = None
        self._red_image = None
        self._left_velocity = None
        self._right_velocity = None
        self.left_pub = rospy.Publisher('lane-follower-left', Float64, queue_size = 1)
        self.right_pub = rospy.Publisher('lane-follower-right', Float64, queue_size = 1)
        self.red_sub = rospy.Subscriber("red-state", Bool, self.update_state)

        self._h = 480
        self._w = 640

        self._state = False
        self._shutdown = False

        self._const = 0.25
        self._gain = 0.2

        self._left = 0.2
        self._right = 0.2
        self._gain = 0.2
        self._ref_vel = 1 

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
            self._left_velocity = 0
            self._right_velocity = 0
            self.pub()
            rospy.sleep(1)

            # Move straight for 5 seconds
            self._left_velocity = 0.5
            self._right_velocity = 0.5
            self.pub()
            rospy.sleep(5)

            # Turn right
            self._left_velocity = 0.5
            self._right_velocity = 0
            self.pub()
            rospy.sleep(0.2)

            # Reset to normal operation
            self.red_line_detected = False

        thread = threading.Thread(target=stop_and_turn)
        thread.start()

    def change_velocity(self, l, r):
        ls = l / (self._w * self._h)
        rs = r / (self._w * self._h)
        self._left_velocity = (self._const + self._gain * (self._left * rs)) * self._ref_vel
        self._right_velocity = (self._const + self._gain * (self._right * ls)) * self._ref_vel
        
        if rs >= ls + 0.1:
            if self._left_velocity > 0.2:
                self._left_velocity = self._left_velocity - 0.2
            self._right_velocity += 0.2

        elif ls >= rs + 0.1:
            if self._right_velocity > 0.2:
                self._right_velocity -= 0.2
            self._right_velocity = 0.2
        
        elif rs < 0.1 and ls < 0.1:
            self._left_velocity = -0.5
            self._right_velocity = -0.5
        
        elif abs(rs - ls) < 0.1:
            self._left_velocity = 0.4
            self._right_velocity = 0.4
        
        self.pub()

    def update_vel(self):
        image_left = self.left_yellow(self._image)
        image_right = self.right_white(self._image)

        l = np.count_nonzero(image_left > 0)
        r = np.count_nonzero(image_right > 0)
        # print(l, r)
        self.change_velocity(l, r)

    def update_state(self, state):
        self._state = state.data

    def pub(self):
        self.left_pub.publish(self._left_velocity)
        self.right_pub.publish(self._right_velocity)  

    def lanefollower(self, msg):
        # convert JPEG bytes to CV image
        self._image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow(self._window, self._image)
        # cv2.waitKey(1)
        # self._left_velocity = 2
        # self._right_velocity = 2
        # print(self._left_velocity)
        # cv2.imshow('left', image_left)
        # cv2.waitKey(1)
        # cv2.imshow('right', image_right)
        # cv2.waitKey(1)
        if self._state == False:
            self.update_vel()
    
    def rescale(a, L, U):
        if np.allclose(L, U):
            return 0.0
        return (a - L) / (U - L)

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


    def left_yellow(self, image):

        luv = cv2.cvtColor(image, cv2.COLOR_BGR2LUV)
        lower_yellow = np.array([0, 0, 170])
        upper_yellow = np.array([2170, 5200, 10000])
        mask_yellow = cv2.inRange(luv, lower_yellow, upper_yellow)
        mask_yellow[ : self._h // 2, : ] = 0
        mask_yellow[ :, self._w // 2 : ] = 0
        # sobel_x = cv2.Sobel(mask_yellow, cv2.CV_64F, 1, 0, ksize=3)
        # sobel_y = cv2.Sobel(mask_yellow, cv2.CV_64F, 0, 1, ksize=3)
        # sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        # sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))
        # threshold_value = 27
        # _, mask_mag = cv2.threshold(sobel_magnitude, threshold_value, 255, cv2.THRESH_BINARY)
        # mask_sobelx_neg = (sobel_x < 0)
        # mask_sobely_neg = (sobel_y < 0)
        # mask_yellow = cv2.GaussianBlur(mask_yellow, (5, 5), 2)
        mask_yellow = cv2.bitwise_and(image, image, mask = mask_yellow)
        mask_yellow = cv2.dilate(mask_yellow, (10, 10), 2)
        # mask_left_edge = np.multiply(self._mask_new, np.multiply(self._mask_left, mask_mag)) # * mask_sobelx_neg * mask_sobely_neg
        # cv2.imshow('left_image', mask_yellow)
        # cv2.waitKey(0)
        return mask_yellow 
    

    def right_white(self, image):

        hsl = cv2.cvtColor(image, cv2.COLOR_BGR2HLS_FULL)
        lower_white = np.array([0, 173, 0])       
        upper_white = np.array([179, 255, 255])   
        mask_white = cv2.inRange(hsl, lower_white, upper_white) 
        mask_white[ : self._h // 2, : ] = 0
        mask_white[ :, : self._w // 2] = 0
        # sobel_x = cv2.Sobel(mask_white, cv2.CV_64F, 1, 0, ksize=3)
        # sobel_y = cv2.Sobel(mask_white, cv2.CV_64F, 0, 1, ksize=3)
        # sobel_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        # sobel_magnitude = np.uint8(255 * sobel_magnitude / np.max(sobel_magnitude))
        # threshold_value = 47
        #_, mask_mag = cv2.threshold(sobel_magnitude, threshold_value, 255, cv2.THRESH_BINARY)
        # mask_white = cv2.GaussianBlur(mask_white, (5, 5), 0)
        # mask_sobelx_pos = (sobel_x > 0)
        # mask_sobely_neg = (sobel_y < 0)
        mask_white = cv2.bitwise_and(image, image, mask = mask_white)
        mask_white = cv2.dilate(mask_white, (10, 10), 2)
        # mask_right_edge = np.multiply(self._mask_new, np.multiply(self._mask_right, mask_mag)) #  * mask_sobelx_pos * mask_sobely_neg
        # cv2.imshow('right_image', mask_white)
        # cv2.waitKey(0)
        return mask_white
    

    def redline(self, msg):
        # convert JPEG bytes to CV image
        self._red_image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # cv2.imshow(self._window, self._image)
        # cv2.waitKey(1)
        self.slowdown(self._red_image)

    def slowdown(self, image):
        self.PID()        
        yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
        lower_red = np.array([0, 0, 170])
        upper_red = np.array([2170, 5200, 10000])
        mask_red = cv2.inRange(yuv, lower_red, upper_red)
        image = cv2.bitwise_and(image, image, mask=mask_red)
        image[ : self._h // 2, : ] = 0
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Find contours in the image
        contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        area = 0
        # Filter horizontal blocks and print "found" when detected
        _, y, w, h, aspect_ratio = 0, 0, 0, 0, 0
        max_contour = max(contours, key=cv2.contourArea)
        # Get bounding box and calculate aspect ratio
        x, y, w, h = cv2.boundingRect(max_contour)
        aspect_ratio = float(w) / h  # width/height
        area = max(area, w * h)
        # print("slowdown")
        red_edge = y + h
        if aspect_ratio > 2:
            if self._ref_vel >= 0.05:
                self._ref_vel = 1 - red_edge / self._w
        else:
            self._ref_vel = 1
        # cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        # cv2.imshow('red', image)
        # cv2.waitKey(0)

        if area > 40000 and self._state == False:
            self._state = True        
        # print(abs(self._ref_vel - self._vel))

    def on_shutdown(self):
        self._left_velocity = 0
        self._right_velocity = 0
        self.pub()

if __name__ == '__main__':
    # create the node
    node = LaneFollowerNode(node_name='lane_follower_node')
    # keep spinning
    rospy.spin()