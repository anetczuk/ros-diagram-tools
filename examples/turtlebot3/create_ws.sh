#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


rm -r -f "src"
unzip -o "$SCRIPT_DIR/turtlebot3-master.zip" -d .
mv -T "turtlebot3-master" "src"


set +u
source /opt/ros/noetic/setup.bash
set -u


catkin config --init
