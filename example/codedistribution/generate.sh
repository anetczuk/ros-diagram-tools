#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../src/rosdiagram"
CHECK_DIR="$TOOL_DIR/.."

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/codedistribution.py --dir $CHECK_DIR --outraw $OUT_DIR/graph.gv.txt --outpng $OUT_DIR/graph.png $@


## dot -Tpng graph.gv.txt -o graph2.png
