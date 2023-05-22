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


set +u
source /opt/ros/noetic/setup.bash
set -u


CURR_DIR="$(pwd)"
cd $CATKIN_DIR

$SCRIPT_DIR/create_ws.sh

echo "clearing catkin workspace"
catkin clean -y

echo "building catkin workspace"
catkin build > "$BUILD_LOG_FILE"

cd $CURR_DIR


set +u
source $CATKIN_DIR/devel/setup.bash
set -u


$TOOL_PATH/dump_cloc.py --cloc_dir "$CATKIN_DIR/src" --out_path "$DUMP_DIR/source_cloc.txt"

$TOOL_PATH/dump_catkin.sh $DUMP_CATKIN_DIR

$SCRIPT_DIR/rosverify.sh
