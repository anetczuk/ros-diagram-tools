#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters -- expected one parameter: {search directory}"
    exit 1
fi

SEARCH_PATH="$1"


## converting images
files_list=$(find $SEARCH_PATH -type f -name "*.puml")

echo "Generating diagrams for files:"
echo "$files_list"

plantuml $files_list -tsvg -nometadata -v
