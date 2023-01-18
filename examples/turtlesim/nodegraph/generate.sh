#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramnode.py -la \
                            --dump_dir $SCRIPT_DIR/../dump/nodeinfo \
                            --topics_dump_dir $SCRIPT_DIR/../dump/topicinfo \
                            --msgs_dump_dir $SCRIPT_DIR/../dump/msginfo \
                            --services_dump_dir $SCRIPT_DIR/../dump/serviceinfo \
                            --srvs_dump_dir $SCRIPT_DIR/../dump/srvinfo \
                            --outpng "$OUT_DIR/graph.png" --outraw "$OUT_DIR/graph.gv.txt" \
                            --outhtml --outdir $OUT_DIR $@


## dot -Tpng graph.gv.txt -o graph2.png
