#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py rosnodegraph -la \
                                          --dump_dir $SCRIPT_DIR/../dump/nodeinfo \
                                          --topics_dump_dir $SCRIPT_DIR/../dump/topicinfo \
                                          --msgs_dump_dir $SCRIPT_DIR/../dump/msginfo \
                                          --services_dump_dir $SCRIPT_DIR/../dump/serviceinfo \
                                          --srvs_dump_dir $SCRIPT_DIR/../dump/srvinfo \
                                          --mainfullgraph \
                                          --outpng "$OUT_DIR/whole_graph.png" --outraw "$OUT_DIR/whole_graph.gv.txt" \
                                          --outhtml --outdir $OUT_DIR $@

cutycapt --url=file://$OUT_DIR/nodes/n__turtlesim.html --out=$OUT_DIR/node-page.png
cutycapt --url=file://$OUT_DIR/nodes/t__turtle1_cmd_vel.html --out=$OUT_DIR/topic-page.png
cutycapt --url=file://$OUT_DIR/nodes/s__spawn.html --out=$OUT_DIR/service-page.png


## dot -Tpng graph.gv.txt -o graph2.png
