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
OUT_ROOT_DIR="$WORK_DIR/out"


rm -rf "$OUT_ROOT_DIR"

mkdir -p "$OUT_ROOT_DIR"


echo "generating codedistribution"
OUT_DIR="$OUT_ROOT_DIR"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py codedistribution --clocdumpdir "$DUMP_DIR/clocsrc" --outpng "$OUT_DIR/codedistribution.png"


echo "generating packagetree"
OUT_DIR="$OUT_ROOT_DIR/catkintree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagetree --catkinlistfile $DUMP_DIR/catkin/list.txt \
                                          --outhtml \
                                          --outmarkdown \
                                          --outdir $OUT_DIR

if [ -f "$OUT_DIR/full_graph.html" ]; then
    cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png
fi
if [ -f "$OUT_DIR/nodes/turtlebot3_bringup.html" ]; then
    cutycapt --url=file://$OUT_DIR/nodes/turtlebot3_bringup.html --out=$OUT_DIR/node-page.png
fi


echo "generating buildtime"
OUT_DIR="$OUT_ROOT_DIR/catkinschedule"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py buildtime --buildlogfile "$DUMP_DIR/build.log.txt" -st 1 -sp 80 \
                                        --outhtml \
                                        --outmarkdown \
                                        --outdir "$OUT_DIR"

$SCRIPT_DIR/../convert_plantuml.sh "$OUT_DIR"
if [ -f "$OUT_DIR/schedule.svg" ]; then
    convert "$OUT_DIR/schedule.svg" -strip -density 600 "$OUT_DIR/schedule.png"
fi
if [ -f "$OUT_DIR/full_graph.html" ]; then
    cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png
fi

                                            
#### classifynodes requires dumped data from main launch file


#### rosnodegraph requires dumped data from running roscore


#### rostopicgraph requires dumped data from running roscore


#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
