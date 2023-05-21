#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"


set +u
source $CATKIN_DIR/devel/setup.bash
set -u


cd $CATKIN_DIR


roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_world.launch
