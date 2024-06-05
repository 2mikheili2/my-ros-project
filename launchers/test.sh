#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
# dt-exec  roslaunch my_package test_node.launch
# rosrun my_package test_node.py

# wait for app to end
dt-launchfile-join