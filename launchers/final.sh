#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
# dt-exec  roslaunch my_package test_node.launch
rosrun my_package new_wheel_encoder_node.py
rosrun my_package new_camera_node.py

# wait for app to end
dt-launchfile-join