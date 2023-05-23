#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py rosnodetree -la \
                                         --dump_dir $SCRIPT_DIR/dump \
                                         --mainfullgraph \
                                         --outdir $OUT_DIR \
                                         --outpng "$OUT_DIR/whole_graph.png" \
                                         --outraw "$OUT_DIR/whole_graph.gv.txt" \
                                         --outhtml $@


## dot -Tpng graph.gv.txt -o graph2.png
