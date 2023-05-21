#!/bin/bash

set -eu
set -m

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
DUMP_DIR="$(pwd)/dump"
DUMP_CATKIN_DIR="$(pwd)/dump/catkin"
DUMP_ROSLAUNCH_DIR="$(pwd)/dump/roslaunch"
BUILD_LOG_FILE="$DUMP_DIR/build.log.txt"


mkdir -p $DUMP_DIR
mkdir -p $DUMP_CATKIN_DIR
mkdir -p $DUMP_ROSLAUNCH_DIR


set +u
source $CATKIN_DIR/devel/setup.bash
set -u


cd $CATKIN_DIR

echo "clearing catkin workspace"
catkin clean -y

echo "building catkin workspace"
catkin build > "$BUILD_LOG_FILE"


$TOOL_PATH/dump_cloc.py --cloc_dir "$CATKIN_DIR/src" --out_path "$DUMP_DIR/source_cloc.txt"

$TOOL_PATH/dump_catkin.sh $DUMP_CATKIN_DIR

$TOOL_PATH/dump_roslaunch.sh "$CATKIN_DIR/src/nexus_4wd_mecanum_gazebo/launch/nexus_4wd_mecanum_world.launch" $DUMP_ROSLAUNCH_DIR


## run roscore
echo "starting roscore"
roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_world.launch &> /tmp/roslaunch.txt &
ROS_PID=$!


terminate() {
    ##
    echo "killing roscore"
    kill $ROS_PID
    fg %1
    exit 0
}

trap terminate INT TERM


echo "waiting for start"
sleep 10
echo ""


## requires running roscore
$TOOL_PATH/dump_ros.sh $DUMP_DIR


terminate