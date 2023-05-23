#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

EXCLUDE_PATH="$SCRIPT_DIR/exclude_topics.txt"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$SCRIPT_DIR/generate_chart.py -la \
                              --rosbagpath "$SCRIPT_DIR/2023-01-16-23-19-09.bag" \
                              --topicsdumppath $SCRIPT_DIR/../dump/topicinfo \
                              --writemessages \
                              --excludelistpath "$EXCLUDE_PATH" \
                              --outdir $OUT_DIR $@

cutycapt --url=file://$OUT_DIR/nodes/_turtlesim_2807_1673907554697.html --out=$OUT_DIR/node-page.png
cutycapt --url=file://$OUT_DIR/msgs/0000751_msg.html --out=$OUT_DIR/message-page.png


files_list=$(find $OUT_DIR -type f -name "*.puml")

echo "Generating diagrams for files:"
echo "$files_list"

plantuml $files_list -tsvg -nometadata -v
