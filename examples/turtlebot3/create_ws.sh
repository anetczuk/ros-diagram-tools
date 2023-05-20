#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters -- expected one parameter: {output directory}"
    exit 1
fi

WORKSPACE_PATH="$1"
CATKIN_WS_PATH="$WORKSPACE_PATH/catkin_ws"


mkdir -p $CATKIN_WS_PATH
cd $CATKIN_WS_PATH


rm -r -f "src"
unzip -o "$SCRIPT_DIR/turtlebot3-master.zip" -d .
mv -T "turtlebot3-master" "src"


set +u
source /opt/ros/noetic/setup.bash
set -u

catkin config --init
