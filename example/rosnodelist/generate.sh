#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../src/rosdiagram"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/rosnodetree.py --dump_dir $SCRIPT_DIR/info --outdir $OUT_DIR --outhtml $@


## dot -Tpng graph.gv.txt -o graph2.png
