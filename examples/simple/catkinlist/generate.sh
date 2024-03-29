#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


rm -rf "$OUT_DIR"

mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py packagetree --catkinlistfile $SCRIPT_DIR/catkin_list.txt \
                                         --descriptionjson "$SCRIPT_DIR/../description.json" \
                                         --outraw $OUT_DIR/graph.gv.txt --outpng $OUT_DIR/graph.png \
                                         --outhtml \
                                         --outmarkdown \
                                         --outdir $OUT_DIR $@


## dot -Tpng graph.gv.txt -o graph2.png
