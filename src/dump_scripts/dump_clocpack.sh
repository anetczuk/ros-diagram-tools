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
    echo "Illegal number of parameters -- expected two parameters: {packages list file} {output directory}"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

PACKAGES_LIST_FILE="$1"
OUT_DIR="$2"

mkdir -p $OUT_DIR


path_to_local() {
        local file_path="$1"
        echo "$OUT_DIR/"$(echo "$file_path" | sed "s/\//_/g")".txt"
}


echo "Dumping data to $OUT_DIR"


LIST_PATH="$OUT_DIR/list.txt"


> "$LIST_PATH"

while IFS= read -r line; do
    IFS=', ' read -r -a line_data <<< "$line"
    src_dir="${line_data[1]}"
    out_file=$(path_to_local "$src_dir") 
    echo "$line -> ${line_data[@]} -> $out_file"
    cloc --sum-one --follow-links "${src_dir}" > "$out_file"
    echo "$src_dir" >> "$LIST_PATH"
done < "$PACKAGES_LIST_FILE"

echo -e "\nDone.\n"
