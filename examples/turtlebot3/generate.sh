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
$TOOL_PATH/rosdiagramtools.py codedistribution --clocpath "$DUMP_DIR/source_cloc.txt" --outpng "$OUT_DIR/codedistribution.png"


echo "generating packagexmltree"
OUT_DIR="$WORK_DIR/out/catkintree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagexmltree -f $DUMP_DIR/catkin/list.txt --outhtml --outdir $OUT_DIR
cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png
cutycapt --url=file://$OUT_DIR/nodes/turtlebot3_bringup.html --out=$OUT_DIR/node-page.png


echo "generating buildtime"
OUT_DIR="$WORK_DIR/out/catkinschedule"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py buildtime -f "$DUMP_DIR/build.log.txt" -st 1 -sp 80 --outhtml --outdir "$OUT_DIR"
$SCRIPT_DIR/../convert_plantuml.sh "$OUT_DIR"
convert "$OUT_DIR/schedule.svg" -strip -density 600 "$OUT_DIR/schedule.png"
cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png

                                            
#### classifynodes requires dumped data from main launch file


#### rosnodegraph requires dumped data from running roscore


#### rostopicgraph requires dumped data from running roscore


#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
