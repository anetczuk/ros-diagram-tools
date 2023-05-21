#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
DUMP_DIR="$(pwd)/dump"
OUT_DIR="$(pwd)/out/catkinschedule_out"
BUILD_LOG_FILE="$DUMP_DIR/build.log.txt"


mkdir -p $DUMP_DIR
mkdir -p $OUT_DIR


set +u
source /opt/ros/noetic/setup.bash
set -u


cd $CATKIN_DIR

echo "clearing catkin workspace"
catkin clean -y

echo "building catkin workspace"
catkin build > "$BUILD_LOG_FILE"


echo "generating data"
$TOOL_PATH/rosdiagramtools.py catkinschedule -f "$BUILD_LOG_FILE" -st 1 -sp 80 --outhtml --outdir "$OUT_DIR"


## converting images
$SCRIPT_DIR/../convert_plantuml.sh "$OUT_DIR"
convert "$OUT_DIR/schedule.svg" -density 600 "$OUT_DIR/build-schedule.png"
