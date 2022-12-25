#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../src"
CHECK_DIR="$TOOL_DIR"

DUMP_DIR="$SCRIPT_DIR/dump"
OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $DUMP_DIR
mkdir -p $OUT_DIR


$TOOL_DIR/dump_cloc.py --cloc_dir $CHECK_DIR --out_path $DUMP_DIR/cloc.txt $@

$TOOL_DIR/rosdiagramcode.py --cloc_path $DUMP_DIR/cloc.txt --highlight "$SCRIPT_DIR/highlight.txt" --outraw $OUT_DIR/graph.gv.txt --outpng $OUT_DIR/graph.png $@


## dot -Tpng graph.gv.txt -o graph2.png
