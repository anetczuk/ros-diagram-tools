#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$TOOL_DIR/rosdiagramtools.py catkinschedule -la \
                                         -f $SCRIPT_DIR/build.log.txt \
                                         -st 1 -sp 150 \
                                         --outhtml --outdir $OUT_DIR $@


files_list=$(find $OUT_DIR -type f -name "*.puml")

echo "Generating diagrams for files:"
echo "$files_list"

plantuml $files_list -tsvg -nometadata -v
