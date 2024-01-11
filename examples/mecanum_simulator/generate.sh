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


rm -rf "$OUT_ROOT_DIR"

mkdir -p "$OUT_ROOT_DIR"


echo "generating codedistribution"
OUT_DIR="$OUT_ROOT_DIR/codedistribution"
mkdir -p "$OUT_DIR"
$TOOL_PATH/rosdiagramtools.py codedistribution --clocdumpdir "$DUMP_DIR/clocdir" \
                                               --outpng "$OUT_DIR/outgraph.png" \
                                               --outhtml \
                                               --outmarkdown \
                                               --outdir "$OUT_DIR"


echo "generating classifynodes"
OUT_DIR="$OUT_ROOT_DIR"
mkdir -p $OUT_DIR
CLASSIFY_NODES_OUT_FILE="$OUT_DIR/nodes_classification.txt"
$TOOL_PATH/rosdiagramtools.py classifynodes --packdumppath $DUMP_DIR/packinfo/list.txt \
                                            --launchdumppath $DUMP_DIR/roslaunch \
                                            --outfile $CLASSIFY_NODES_OUT_FILE


echo "generating catkin deps tree"
CATKIN_PKG_OUT_DIR="$OUT_ROOT_DIR/catkintree"
mkdir -p $CATKIN_PKG_OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagetree --catkinlistfile $DUMP_DIR/catkindeps/list.txt \
                                          --highlightitems $DUMP_DIR/catkindeps/packages.txt \
                                          --outhtml \
                                          --outmarkdown \
                                          --outdir $CATKIN_PKG_OUT_DIR
## generate image from html
if [ -f "$CATKIN_PKG_OUT_DIR/full_graph.html" ]; then
    cutycapt --url=file://$CATKIN_PKG_OUT_DIR/full_graph.html --out=$CATKIN_PKG_OUT_DIR/main-page.png
fi
if [ -f "$CATKIN_PKG_OUT_DIR/nodes/nexus_4wd_mecanum_gazebo.html" ]; then
    cutycapt --url=file://$CATKIN_PKG_OUT_DIR/nodes/nexus_4wd_mecanum_gazebo.html --out=$CATKIN_PKG_OUT_DIR/node-page.png
fi


echo "generating buildtime"
BUILD_OUT_DIR="$OUT_ROOT_DIR/catkinschedule"
mkdir -p $BUILD_OUT_DIR
$TOOL_PATH/rosdiagramtools.py buildtime --buildlogfile "$DUMP_DIR/build.log.txt" -st 1 -sp 80 \
                                        --outhtml \
                                        --outmarkdown \
                                        --outdir "$BUILD_OUT_DIR"

$SCRIPT_DIR/../convert_plantuml.sh "$BUILD_OUT_DIR"
if [ -f "$BUILD_OUT_DIR/schedule.svg" ]; then
    convert "$BUILD_OUT_DIR/schedule.svg" -strip -density 600 "$BUILD_OUT_DIR/schedule.png"
fi
if [ -f "$BUILD_OUT_DIR/full_graph.html" ]; then
    cutycapt --url=file://$BUILD_OUT_DIR/full_graph.html --out=$BUILD_OUT_DIR/main-page.png
fi


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
                                           --outpng "$NODE_OUT_DIR/whole_graph.png" \
                                           --outraw "$NODE_OUT_DIR/graph.gv.txt"


echo "running rostopicgraph"
TOPIC_OUT_DIR="$OUT_ROOT_DIR/topictree"
mkdir -p $TOPIC_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rostopicgraph --topicsdumppath $DUMP_DIR/topicinfo \
                                            --outraw $TOPIC_OUT_DIR/graph.gv.txt \
                                            --outpng $TOPIC_OUT_DIR/graph.png


echo "running index"
INDEX_OUT_DIR="$OUT_ROOT_DIR/index"
mkdir -p $INDEX_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosindex --customlist "src code distribution" $OUT_DIR/codedistribution/full_graph.autolink \
                                                    "catkin pkg tree" $CATKIN_PKG_OUT_DIR/full_graph.autolink \
                                                    "catkin build" $BUILD_OUT_DIR/full_graph.autolink \
                                                    "nodes classify" $CLASSIFY_NODES_OUT_FILE \
                                                    "nodes graph" $NODE_OUT_DIR/whole_graph.png \
                                                    "topics graph" $TOPIC_OUT_DIR/graph.png \
                                       --outhtml \
                                       --outmarkdown \
                                       --outdir $INDEX_OUT_DIR

if [ -f "$INDEX_OUT_DIR/main_page.html" ]; then
    cutycapt --url=file://$INDEX_OUT_DIR/main_page.html --out=$INDEX_OUT_DIR/main-page.png
fi


echo "running general script"
GENERAL_OUT_DIR="$OUT_ROOT_DIR/general"
mkdir -p $GENERAL_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosgeneral --dumprootdir $DUMP_DIR \
                                         --launchdumppath $DUMP_DIR/roslaunch \
                                         --classifynodesfile $CLASSIFY_NODES_OUT_FILE \
                                         --pkgsfilterlist $DUMP_DIR/catkindeps/packages.txt \
                                         --includerosinternals \
                                         --descriptionjsonfile $SCRIPT_DIR/description.json \
                                         --highlightnodeslist $SCRIPT_DIR/highlightnodes.txt \
                                         --highlightpackageslist $SCRIPT_DIR/highlightpackages.txt \
                                         --customlist "catkin build view" $BUILD_OUT_DIR/full_graph.autolink \
                                         --outhtml \
                                         --outmarkdown \
                                         --outdir $GENERAL_OUT_DIR

if [ -f "$GENERAL_OUT_DIR/main_page.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/main_page.html --out=$GENERAL_OUT_DIR/main-page.png
fi
if [ -f "$GENERAL_OUT_DIR/paramview/main_page.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/paramview/main_page.html --out=$GENERAL_OUT_DIR/paramview-page.png
fi
if [ -f "$GENERAL_OUT_DIR/msgview/main_page.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/msgview/main_page.html --out=$GENERAL_OUT_DIR/msgview-page.png
fi
if [ -f "$GENERAL_OUT_DIR/nodeview/full_graph.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/full_graph.html --out=$GENERAL_OUT_DIR/nodeview-main-page.png
fi
if [ -f "$GENERAL_OUT_DIR/nodeview/nodes/n__gazebo.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/nodes/n__gazebo.html --out=$GENERAL_OUT_DIR/nodeview-node-page.png
fi
if [ -f "$GENERAL_OUT_DIR/nodeview/nodes/t__clock.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/nodes/t__clock.html --out=$GENERAL_OUT_DIR/nodeview-topic-page.png
fi
if [ -f "$GENERAL_OUT_DIR/nodeview/nodes/s__gazebo_set_model_state.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/nodeview/nodes/s__gazebo_set_model_state.html --out=$GENERAL_OUT_DIR/nodeview-service-page.png
fi
if [ -f "$GENERAL_OUT_DIR/packageview/full_graph.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/packageview/full_graph.html --out=$GENERAL_OUT_DIR/packageview-main-page.png
fi

if [ -f "$GENERAL_OUT_DIR/launchview/full_graph.html" ]; then
    cutycapt --url=file://$GENERAL_OUT_DIR/launchview/full_graph.html --out=$GENERAL_OUT_DIR/launchview-main-page.png
fi
launch_hml_file="$GENERAL_OUT_DIR/launchview/launches/_home_vbox_rosdiagrams_mecanum_catkin_ws_src_nexus_4wd_mecanum_gazebo_launch_nexus_4wd_mecanum_world.launch.html"
if [ -f "$launch_hml_file" ]; then
    cutycapt --url=file://$launch_hml_file --out=$GENERAL_OUT_DIR/launchview-file-page.png
fi

#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
