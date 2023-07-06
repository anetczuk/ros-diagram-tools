#!/bin/bash

set -eu
set -m

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
DUMP_DIR="$(pwd)/dump"
DUMP_CATKIN_DIR="$(pwd)/dump/catkin"
BUILD_LOG_FILE="$DUMP_DIR/build.log.txt"


mkdir -p $CATKIN_DIR
mkdir -p $DUMP_DIR
mkdir -p $DUMP_CATKIN_DIR


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

$TOOL_PATH/rosdiagramdump.py dumpclocdir --clocrundir "$CATKIN_DIR/src" --outdir "$DUMP_DIR/clocsrc"

cd $CATKIN_DIR
$TOOL_PATH/rosdiagramdump.py dumpcatkindeps --outdir $DUMP_CATKIN_DIR
cd $CURR_DIR

$SCRIPT_DIR/rosverify.sh
