#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../src/rosdiagram"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/catkintree.py --file $SCRIPT_DIR/catkin_list.txt --outraw $OUT_DIR/graph.gv.txt --outpng $OUT_DIR/graph.png $@


## dot -Tpng graph.gv.txt -o graph2.png
