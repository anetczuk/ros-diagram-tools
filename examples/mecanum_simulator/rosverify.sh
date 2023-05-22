#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
OUT_DIR="$(pwd)/out"


mkdir -p $OUT_DIR


set +u
source $CATKIN_DIR/devel/setup.bash
set -u


cd $CATKIN_DIR


$TOOL_PATH/rosdiagramtools.py rosverify -w . &> $OUT_DIR/rosverify.txt

cat $OUT_DIR/rosverify.txt
