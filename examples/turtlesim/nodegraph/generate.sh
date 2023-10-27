#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


rm -rf "$OUT_DIR"

mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py rosnodegraph -la \
                                          --nodesdumppath $SCRIPT_DIR/../dump/nodeinfo \
                                          --topicsdumppath $SCRIPT_DIR/../dump/topicinfo \
                                          --msgsdumppath $SCRIPT_DIR/../dump/msginfo \
                                          --servicesdumppath $SCRIPT_DIR/../dump/serviceinfo \
                                          --srvsdumppath $SCRIPT_DIR/../dump/srvinfo \
                                          --mainfullgraph \
                                          --outpng "$OUT_DIR/whole_graph.png" --outraw "$OUT_DIR/whole_graph.gv.txt" \
                                          --outhtml \
                                          --outmarkdown \
                                          --outdir $OUT_DIR $@

if [ -f "$OUT_DIR/nodes/n__turtlesim.html" ]; then
    cutycapt --url=file://$OUT_DIR/nodes/n__turtlesim.html --out=$OUT_DIR/node-page.png
fi
if [ -f "$OUT_DIR/nodes/t__turtle1_cmd_vel.html" ]; then
    cutycapt --url=file://$OUT_DIR/nodes/t__turtle1_cmd_vel.html --out=$OUT_DIR/topic-page.png
fi
if [ -f "$OUT_DIR/nodes/s__spawn.html" ]; then
    cutycapt --url=file://$OUT_DIR/nodes/s__spawn.html --out=$OUT_DIR/service-page.png
fi


## dot -Tpng graph.gv.txt -o graph2.png
