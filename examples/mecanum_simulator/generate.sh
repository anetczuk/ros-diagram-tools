#!/bin/bash

set -eu
set -x


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


WORK_DIR="$(pwd)"
if [ "$#" -ge 1 ]; then
    WORK_DIR="$1"    
fi

TOOL_PATH="$SCRIPT_DIR/../../src"

DUMP_DIR="$WORK_DIR/dump"
OUT_ROOT_DIR="$WORK_DIR/out"


# rm -rf "$OUT_ROOT_DIR"


echo "generating codedistribution"
OUT_DIR="$OUT_ROOT_DIR"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py codedistribution --clocdumpdir "$DUMP_DIR/clocdir" \
                                               --outpng "$OUT_DIR/codedistribution/outgraph.png" --outdir "$OUT_DIR/codedistribution"


echo "generating classifynodes"
OUT_DIR="$OUT_ROOT_DIR"
mkdir -p $OUT_DIR
CLASSIFY_NODES_OUT_FILE="$OUT_DIR/nodes_classification.txt"
$TOOL_PATH/rosdiagramtools.py classifynodes --packdumppath $DUMP_DIR/packinfo/list.txt \
                                            --launchdumppath $DUMP_DIR/roslaunch \
                                            --outfile $CLASSIFY_NODES_OUT_FILE


echo "generating packagetree"
CATKIN_PKG_OUT_DIR="$OUT_ROOT_DIR/catkintree"
mkdir -p $CATKIN_PKG_OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagetree --catkinlistfile $DUMP_DIR/catkindeps/list.txt \
                                          --highlightitems $DUMP_DIR/catkindeps/packages.txt \
                                          --outhtml --outdir $CATKIN_PKG_OUT_DIR
## generate image from html
cutycapt --url=file://$CATKIN_PKG_OUT_DIR/full_graph.html --out=$CATKIN_PKG_OUT_DIR/main-page.png
cutycapt --url=file://$CATKIN_PKG_OUT_DIR/nodes/nexus_4wd_mecanum_gazebo.html --out=$CATKIN_PKG_OUT_DIR/node-page.png


echo "generating buildtime"
BUILD_OUT_DIR="$OUT_ROOT_DIR/catkinschedule"
mkdir -p $BUILD_OUT_DIR
$TOOL_PATH/rosdiagramtools.py buildtime --buildlogfile "$DUMP_DIR/build.log.txt" -st 1 -sp 80 --outhtml --outdir "$BUILD_OUT_DIR"
$SCRIPT_DIR/../convert_plantuml.sh "$BUILD_OUT_DIR"
convert "$BUILD_OUT_DIR/schedule.svg" -strip -density 600 "$BUILD_OUT_DIR/schedule.png"
cutycapt --url=file://$BUILD_OUT_DIR/full_graph.html --out=$BUILD_OUT_DIR/main-page.png


echo "running rosnodegraph"
NODE_OUT_DIR="$OUT_ROOT_DIR/nodetree"
mkdir -p $NODE_OUT_DIR

## generate main graph
$TOOL_PATH/rosdiagramtools.py rosnodegraph -la \
                                           --nodesdumppath $DUMP_DIR/nodeinfo \
                                           --topicsdumppath $DUMP_DIR/topicinfo \
                                           --msgsdumppath $DUMP_DIR/msginfo \
                                           --servicesdumppath $DUMP_DIR/serviceinfo \
                                           --srvsdumppath $DUMP_DIR/srvinfo \
                                           --mainfullgraph \
                                           --includerosinternals \
                                           --outpng "$NODE_OUT_DIR/whole_graph.png" --outraw "$NODE_OUT_DIR/graph.gv.txt"


echo "running rostopicgraph"
TOPIC_OUT_DIR="$OUT_ROOT_DIR/topictree"
mkdir -p $TOPIC_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rostopicgraph --topicsdumppath $DUMP_DIR/topicinfo \
                                            --outraw $TOPIC_OUT_DIR/graph.gv.txt \
                                            --outpng $TOPIC_OUT_DIR/graph.png


echo "running index"
INDEX_OUT_DIR="$OUT_ROOT_DIR/index"
mkdir -p $INDEX_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosindex --customlist "src code distribution" $OUT_DIR/codedistribution/full_graph.html \
                                                    "catkin pkg tree" $CATKIN_PKG_OUT_DIR/full_graph.html \
                                                    "catkin build" $BUILD_OUT_DIR/full_graph.html \
                                                    "nodes classify" $CLASSIFY_NODES_OUT_FILE \
                                                    "nodes graph" $NODE_OUT_DIR/whole_graph.png \
                                                    "topics graph" $TOPIC_OUT_DIR/graph.png \
                                       --outdir $INDEX_OUT_DIR

cutycapt --url=file://$INDEX_OUT_DIR/main_page.html --out=$INDEX_OUT_DIR/main-page.png


echo "running general script"
GENERAL_OUT_DIR="$OUT_ROOT_DIR/general"
mkdir -p $GENERAL_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosgeneral --dumprootdir $DUMP_DIR \
                                         --classifynodesfile $CLASSIFY_NODES_OUT_FILE \
                                         --pkgsfilterlist $DUMP_DIR/catkindeps/packages.txt \
                                         --includerosinternals \
                                         --outdir $GENERAL_OUT_DIR

cutycapt --url=file://$GENERAL_OUT_DIR/main_page.html --out=$GENERAL_OUT_DIR/main-page.png
cutycapt --url=file://$GENERAL_OUT_DIR/paramview/main_page.html --out=$GENERAL_OUT_DIR/paramview-page.png
cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/full_graph.html --out=$GENERAL_OUT_DIR/nodeview-main-page.png
cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/nodes/n__gazebo.html --out=$GENERAL_OUT_DIR/nodeview-node-page.png
cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/nodes/t__clock.html --out=$GENERAL_OUT_DIR/nodeview-topic-page.png
cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/nodes/s__gazebo_set_model_state.html --out=$GENERAL_OUT_DIR/nodeview-service-page.png
cutycapt --url=file://$GENERAL_OUT_DIR/packageview/full_graph.html --out=$GENERAL_OUT_DIR/packageview-main-page.png


#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
