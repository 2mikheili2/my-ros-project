#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
dt-exec  roslaunch my_package lane_follower.launch
# rosrun my_package new_node.py

# wait for app to end
dt-launchfile-join