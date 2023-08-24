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
$TOOL_PATH/rosdiagramtools.py codedistribution --clocdumpdir "$DUMP_DIR/clocdir" --outpng "$OUT_DIR/codedistribution_src.png"
$TOOL_PATH/rosdiagramtools.py codedistribution --clocdumpdir "$DUMP_DIR/clocpackinfo" \
                                               --filteritems $DUMP_DIR/catkindeps/packages.txt \
                                               --outpng "$OUT_DIR/codedistribution_pack.png"
#$TOOL_PATH/rosdiagramtools.py codedistribution --clocjsonpath "$DUMP_DIR/source_cloc.txt" --outpng "$OUT_DIR/codedistribution_json.png"


echo "generating classifynodes"
OUT_DIR="$OUT_ROOT_DIR"
mkdir -p $OUT_DIR
CLASSIFY_NODES_OUT_FILE="$OUT_DIR/nodes_classification.txt"
$TOOL_PATH/rosdiagramtools.py classifynodes --packdumppath $DUMP_DIR/packinfo/list.txt \
                                            --launchdumppath $DUMP_DIR/roslaunch \
                                            --outfile $CLASSIFY_NODES_OUT_FILE


echo "generating packagetree"
OUT_DIR="$OUT_ROOT_DIR/catkintree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagetree --catkinlistfile $DUMP_DIR/catkindeps/list.txt \
                                          --highlightitems $DUMP_DIR/catkindeps/packages.txt \
                                          --outhtml --outdir $OUT_DIR
## generate image from html
cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png
cutycapt --url=file://$OUT_DIR/nodes/nexus_4wd_mecanum_gazebo.html --out=$OUT_DIR/node-page.png

PKG_OUT_DIR="$OUT_ROOT_DIR/packagestree"
mkdir -p $PKG_OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagetree --packdumppath $DUMP_DIR/packinfo \
                                          --classifynodesfile $CLASSIFY_NODES_OUT_FILE \
                                          --highlightitems $DUMP_DIR/catkindeps/packages.txt \
                                          --topitems $DUMP_DIR/catkindeps/packages.txt \
                                          --outhtml --outdir $PKG_OUT_DIR
cutycapt --url=file://$PKG_OUT_DIR/full_graph.html --out=$PKG_OUT_DIR/main-page.png
#cutycapt --url=file://$PKG_OUT_DIR/nodes/nexus_4wd_mecanum_gazebo.html --out=$PKG_OUT_DIR/node-page.png


echo "generating buildtime"
BUILD_OUT_DIR="$OUT_ROOT_DIR/catkinschedule"
mkdir -p $BUILD_OUT_DIR
$TOOL_PATH/rosdiagramtools.py buildtime --buildlogfile "$DUMP_DIR/build.log.txt" -st 1 -sp 80 --outhtml --outdir "$BUILD_OUT_DIR"
$SCRIPT_DIR/../convert_plantuml.sh "$BUILD_OUT_DIR"
convert "$BUILD_OUT_DIR/schedule.svg" -strip -density 600 "$BUILD_OUT_DIR/schedule.png"
cutycapt --url=file://$BUILD_OUT_DIR/full_graph.html --out=$BUILD_OUT_DIR/main-page.png


echo "running rosparamlist"
PARAMS_OUT_DIR="$OUT_ROOT_DIR/paramslist"
mkdir -p $PARAMS_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosparamlist -la \
                                           --dumpyamlfile $DUMP_DIR/paraminfo/params.yml \
                                           --outdir $PARAMS_OUT_DIR
cutycapt --url=file://$PARAMS_OUT_DIR/main_page.html --out=$PARAMS_OUT_DIR/main-page.png


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

## generate HML graph
$TOOL_PATH/rosdiagramtools.py rosnodegraph -la \
                                           --nodesdumppath $DUMP_DIR/nodeinfo \
                                           --topicsdumppath $DUMP_DIR/topicinfo \
                                           --msgsdumppath $DUMP_DIR/msginfo \
                                           --servicesdumppath $DUMP_DIR/serviceinfo \
                                           --srvsdumppath $DUMP_DIR/srvinfo \
                                           --classifynodesfile $CLASSIFY_NODES_OUT_FILE \
                                           --includerosinternals \
                                           --outhtml --outdir $NODE_OUT_DIR

cutycapt --url=file://$NODE_OUT_DIR/full_graph.html --out=$NODE_OUT_DIR/main-page.png
cutycapt --url=file://$NODE_OUT_DIR/nodes/n__gazebo.html --out=$NODE_OUT_DIR/node-page.png
cutycapt --url=file://$NODE_OUT_DIR/nodes/t__clock.html --out=$NODE_OUT_DIR/topic-page.png
cutycapt --url=file://$NODE_OUT_DIR/nodes/s__gazebo_set_model_state.html --out=$NODE_OUT_DIR/service-page.png


echo "running rostopicgraph"
TOPIC_OUT_DIR="$OUT_ROOT_DIR/topictree"
mkdir -p $TOPIC_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rostopicgraph --topicsdumppath $DUMP_DIR/topicinfo \
                                            --outraw $TOPIC_OUT_DIR/graph.gv.txt \
                                            --outpng $TOPIC_OUT_DIR/graph.png


echo "running index"
INDEX_OUT_DIR="$OUT_ROOT_DIR/index"
mkdir -p $INDEX_OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosindex --packagesview $PKG_OUT_DIR/full_graph.html \
                                       --paramsview $PARAMS_OUT_DIR/main_page.html \
                                       --nodesview $NODE_OUT_DIR/full_graph.html \
                                       --customlist "catkin build" $BUILD_OUT_DIR/full_graph.html \
                                       --outdir $INDEX_OUT_DIR

cutycapt --url=file://$INDEX_OUT_DIR/main_page.html --out=$INDEX_OUT_DIR/index_page.png


#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
