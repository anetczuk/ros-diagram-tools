#!/bin/bash
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

##
## Dump cloc over list of packages
##

##set -eu
set -e


if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters -- expected two parameters: {scan directory} {output directory}"
    exit 1
fi


## check if cloc is installed
cloc --version > /dev/null

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

SCAN_DIR="$1"
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


LIST_PATH="$OUT_DIR/list.txt"


> "$LIST_PATH"

for dir in ${SCAN_DIR}/*/; do
    src_dir=$(realpath "$dir")
    out_file=$(path_to_local "$src_dir")
    echo "$src_dir -> $out_file"
    ## cloc will fail on broken symlinks, so prevent stopping the script
    cloc --sum-one --follow-links "${src_dir}" > "$out_file" || true
    echo "$src_dir" >> "$LIST_PATH"
done

echo -e "\nDone.\n"
