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


echo "generating packagetree"
OUT_DIR="$WORK_DIR/out/catkintree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py packagetree -f $DUMP_DIR/catkin/list.txt --outhtml --outdir $OUT_DIR
cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png
cutycapt --url=file://$OUT_DIR/nodes/nexus_4wd_mecanum_gazebo.html --out=$OUT_DIR/node-page.png


echo "generating catkinschedule"
OUT_DIR="$WORK_DIR/out/catkinschedule"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py catkinschedule -f "$DUMP_DIR/build.log.txt" -st 1 -sp 80 --outhtml --outdir "$OUT_DIR"
$SCRIPT_DIR/../convert_plantuml.sh "$OUT_DIR"
convert "$OUT_DIR/schedule.svg" -density 600 "$OUT_DIR/schedule.png"
cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png


echo "generating classifynodes"
OUT_DIR="$WORK_DIR/out"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py classifynodes --pack_list_file $DUMP_DIR/packinfo/list.txt \
                                            --launch_dump_dir $DUMP_DIR/roslaunch \
                                            --out_file $OUT_DIR/nodes_classification.txt


echo "running rosnodetree"
OUT_DIR="$WORK_DIR/out/nodetree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py rosnodetree -la \
                                          --dump_dir $DUMP_DIR/nodeinfo \
                                          --topics_dump_dir $DUMP_DIR/topicinfo \
                                          --msgs_dump_dir $DUMP_DIR/msginfo \
                                          --services_dump_dir $DUMP_DIR/serviceinfo \
                                          --srvs_dump_dir $DUMP_DIR/srvinfo \
                                          --includerosinternals \
                                          --outpng "$OUT_DIR/whole_graph.png" --outraw "$OUT_DIR/graph.gv.txt" \
                                          --outhtml --outdir $OUT_DIR
cutycapt --url=file://$OUT_DIR/full_graph.html --out=$OUT_DIR/main-page.png
cutycapt --url=file://$OUT_DIR/nodes/n__gazebo.html --out=$OUT_DIR/node-page.png
cutycapt --url=file://$OUT_DIR/nodes/t__clock.html --out=$OUT_DIR/topic-page.png
cutycapt --url=file://$OUT_DIR/nodes/s__gazebo_set_model_state.html --out=$OUT_DIR/service-page.png


echo "running rostopictree"
OUT_DIR="$WORK_DIR/out/topictree"
mkdir -p $OUT_DIR
$TOOL_PATH/rosdiagramtools.py rostopictree --dump_dir $DUMP_DIR/topicinfo \
                                           --outraw $OUT_DIR/graph.gv.txt \
                                           --outpng $OUT_DIR/graph.png

                                            
#### rosbagflow requires rosbag file


#### rosverify works directly on source directory


echo "done"
