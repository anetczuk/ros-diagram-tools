#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"

EXCLUDE_PATH="$SCRIPT_DIR/exclude_topics.txt"

OUT_DIR="$SCRIPT_DIR/out"


mkdir -p $OUT_DIR


$SCRIPT_DIR/generate_chart.py -la \
                            --bag_path "$SCRIPT_DIR/2023-01-16-23-19-09.bag" \
                            --topic_dump_dir $SCRIPT_DIR/../dump/topicinfo \
                            --write_messages \
                            --exclude_list_path "$EXCLUDE_PATH" \
                            --outdir $OUT_DIR $@


files_list=$(find $OUT_DIR -type f -name "*.puml")

echo "Generating diagrams for files:"
echo "$files_list"

plantuml $files_list -tsvg -nometadata -v
