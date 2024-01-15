#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"
CHECK_DIR="$TOOL_DIR"

DUMP_DIR="$SCRIPT_DIR/dump"
OUT_DIR="$SCRIPT_DIR/out"


rm -rf "$OUT_DIR"

mkdir -p $DUMP_DIR
mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py roslaunchgraph --launchfile $DUMP_DIR/launch.json \
                                            --outraw $OUT_DIR/raw.txt \
                                            --outpng $OUT_DIR/graph.png $@


$TOOL_DIR/rosdiagramtools.py roslaunchgraph --launchfile $DUMP_DIR/launch.json \
                                            --outhtml --outmarkdown \
                                            --outdir $OUT_DIR $@
