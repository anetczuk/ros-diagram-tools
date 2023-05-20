#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"

CATKIN_DIR="$(pwd)/catkin_ws"
DUMP_DIR="$(pwd)/dump"
OUT_DIR="$(pwd)/out"


mkdir -p $DUMP_DIR
mkdir -p $OUT_DIR


echo "dumping data"
$TOOL_PATH/dump_cloc.py --cloc_dir "$CATKIN_DIR/src" --out_path "$DUMP_DIR/source_cloc.txt"

echo "generating data"
$TOOL_PATH/rosdiagramtools.py codedistribution --cloc_path "$DUMP_DIR/source_cloc.txt" --outpng "$OUT_DIR/codedistribution.png"
