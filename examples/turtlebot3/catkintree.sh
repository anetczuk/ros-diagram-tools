#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
DUMP_DIR="$(pwd)/dump/dump_catkin"
OUT_DIR="$(pwd)/out/catkintree_out"


mkdir -p $DUMP_DIR
mkdir -p $OUT_DIR


set +u
source /opt/ros/noetic/setup.bash
set -u


cd $CATKIN_DIR

echo "building catkin workspace"
catkin build


$TOOL_PATH/dump_catkin.sh $DUMP_DIR

echo "generating data"
$TOOL_PATH/rosdiagramtools.py catkintree -f $DUMP_DIR/list.txt --outhtml --outdir $OUT_DIR
