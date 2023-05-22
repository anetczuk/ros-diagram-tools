#!/bin/bash

set -eu
set -x


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


WORK_DIR="$(pwd)"
if [ "$#" -ge 1 ]; then
    WORK_DIR="$1"    
fi

TOOL_PATH="$SCRIPT_DIR/../../src/"

DUMP_DIR="$WORK_DIR/dump"


echo "generating codedistribution"
OUT_DIR="$WORK_DIR/out"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py codedistribution --cloc_path "$DUMP_DIR/source_cloc.txt" --outpng "$OUT_DIR/codedistribution.png"


echo "generating catkintree"
OUT_DIR="$WORK_DIR/out/catkintree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py catkintree -f $DUMP_DIR/catkin/list.txt --outhtml --outdir $OUT_DIR


echo "generating catkinschedule"
OUT_DIR="$WORK_DIR/out/catkinschedule"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py catkinschedule -f "$DUMP_DIR/build.log.txt" -st 1 -sp 80 --outhtml --outdir "$OUT_DIR"
$SCRIPT_DIR/../convert_plantuml.sh "$OUT_DIR"
convert "$OUT_DIR/schedule.svg" -density 600 "$OUT_DIR/schedule.png"

                                            
#### classifynodes requires dumped data from main launch file


#### rostopictree requires dumped data from running roscore


#### rosnodetree requires dumped data from running roscore


#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
