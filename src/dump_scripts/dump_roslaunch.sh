#!/bin/bash
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

##
## Dump rostopic info to files
##

set -eu
#set -x


if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters -- expected two parameters: {entry launch file} {output directory}"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

LAUNCH_FILE="$1"
OUT_DIR="$2"

mkdir -p $OUT_DIR


path_to_local() {
        ## have to match Python function "showgraph.io.prepare_filesystem_name()"
        local file_path="$1"
        file_path=$(echo "$file_path" | sed "s/\//_/g")
        file_path=$(echo "$file_path" | sed "s/|/_/g")
        file_path=$(echo "$file_path" | sed "s/-/_/g")
        echo "$OUT_DIR/${file_path}.txt"
}


echo "Dumping data to $OUT_DIR"


LIST_FILE_PATH="$OUT_DIR/list.txt"


roslaunch $LAUNCH_FILE --files > "$LIST_FILE_PATH"

sort -o "$LIST_FILE_PATH" "$LIST_FILE_PATH"

## removing old files
while read launch_file_output; do
    #echo "$launch_file_output"
    launch_file=$(path_to_local "$launch_file_output")
    rm $launch_file || true
done < "$LIST_FILE_PATH"


## dumping nodes data
nodes_list=$(roslaunch --nodes $LAUNCH_FILE)

for item in $nodes_list; do
    node_file=$(roslaunch $LAUNCH_FILE --find-node="$item")
    echo "$item: $node_file"

    out_file=$(path_to_local "$node_file") 
    echo $item >> $out_file
done

echo -e "\nDone.\n"
