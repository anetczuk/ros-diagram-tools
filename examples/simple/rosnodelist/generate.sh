#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py rosnodegraph -la \
                                          --nodesdumppath $SCRIPT_DIR/dump \
                                          --mainfullgraph \
                                          --descriptionjson "$SCRIPT_DIR/../description.json" \
                                          --highlightitems "$SCRIPT_DIR/../highlightnodes.txt" \
                                          --outdir $OUT_DIR \
                                          --outpng "$OUT_DIR/whole_graph.png" \
                                          --outraw "$OUT_DIR/whole_graph.gv.txt" \
                                          --outhtml $@


## dot -Tpng graph.gv.txt -o graph2.png
