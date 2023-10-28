#!/bin/bash


##
## Configure catkin, build project, start roscore and dump data.
##


set -eux
set -m

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
DUMP_DIR="$(pwd)/dump"
DUMP_CATKIN_DIR="$(pwd)/dump/catkindeps"
DUMP_ROSLAUNCH_DIR="$(pwd)/dump/roslaunch"
BUILD_LOG_FILE="$DUMP_DIR/build.log.txt"


mkdir -p $CATKIN_DIR
mkdir -p $DUMP_DIR
mkdir -p $DUMP_CATKIN_DIR
mkdir -p $DUMP_ROSLAUNCH_DIR


CURR_DIR="$(pwd)"


## rebuild project

catkin_rebuild() {
    set +u
    source /opt/ros/noetic/setup.bash
    set -u

    cd $CATKIN_DIR

    $SCRIPT_DIR/create_ws.sh

    echo "clearing catkin workspace"
    catkin clean -y
    
    echo "building catkin workspace"
    catkin build > "$BUILD_LOG_FILE"
}

catkin_rebuild


## dump data

set +u
source $CATKIN_DIR/devel/setup.bash
set -u

cd $CURR_DIR

$TOOL_PATH/rosdiagramdump.py dumpclocdir --clocrundir "$CATKIN_DIR/src" --outdir "$DUMP_DIR/clocdir"

cd $CATKIN_DIR
$TOOL_PATH/rosdiagramdump.py dumpcatkindeps --outdir $DUMP_CATKIN_DIR
cd $CURR_DIR

$SCRIPT_DIR/rosverify.sh

$TOOL_PATH/rosdiagramdump.py dumproslaunch --launchfile "$CATKIN_DIR/src/nexus_4wd_mecanum_gazebo/launch/nexus_4wd_mecanum_world.launch" \
                                           --outdir $DUMP_ROSLAUNCH_DIR


## run roscore
echo "starting roscore"
roslaunch nexus_4wd_mecanum_gazebo nexus_4wd_mecanum_world.launch &> /tmp/roslaunch.txt &
ROS_PID=$!


terminate_roscore() {
    ##
    echo "killing roscore"
    kill $ROS_PID
    fg %1
    exit 0
}

trap terminate_roscore INT TERM


echo "waiting for start"
sleep 10
echo ""


## requires running roscore
$TOOL_PATH/rosdiagramdump.py dumprosrelative --outdir $DUMP_DIR
#$TOOL_PATH/rosdiagramdump.py dumpros --outdir $DUMP_DIR


terminate_roscore
