#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_PATH="$SCRIPT_DIR/../../src/"


DUMP_DIR="$SCRIPT_DIR/dump"
OUT_DIR="$SCRIPT_DIR/out"
$TOOL_PATH/rosdiagramtools.py codedistribution --cloc_path "$DUMP_DIR/source_cloc.txt" --outpng "$OUT_DIR/codedistribution.png"


DUMP_DIR="$SCRIPT_DIR/dump/dump_catkin"
OUT_DIR="$SCRIPT_DIR/out/catkintree_out"
$TOOL_PATH/rosdiagramtools.py catkintree -f $DUMP_DIR/list.txt --outhtml --outdir $OUT_DIR


DUMP_DIR="$SCRIPT_DIR/dump"
OUT_DIR="$SCRIPT_DIR/out/catkinschedule_out"
BUILD_LOG_FILE="$DUMP_DIR/build.log.txt"
$TOOL_PATH/rosdiagramtools.py catkinschedule -f "$BUILD_LOG_FILE" -st 1 -sp 80 --outhtml --outdir "$OUT_DIR"
$SCRIPT_DIR/../convert_plantuml.sh "$OUT_DIR"
convert "$OUT_DIR/schedule.svg" -density 600 "$OUT_DIR/build-schedule.png"


#### rosverify works directly on source directory
