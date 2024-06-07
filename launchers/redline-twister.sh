#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher

dt-exec  roslaunch my_package redline-twister.launch

# wait for app to end
dt-launchfile-join