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


if [ "$#" -lt 1 ]; then
    echo "Illegal number of parameters -- expected one parameter (output directory)"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

INFO_DIR="$1"

mkdir -p $INFO_DIR


echo "Dumping data to $INFO_DIR"


if [[ $* == *--listprovided* ]]; then
    ## read data from already provided list file
    echo "loading data from provided list"
else
    rosmsg list > "$INFO_DIR/list.txt"
fi

sort -o "$INFO_DIR/list.txt" "$INFO_DIR/list.txt"

items_list=$(<"$INFO_DIR/list.txt")


for item in $items_list; do
    out_file="$INFO_DIR/"$(echo "$item" | sed "s/\//_/g")".txt"
    echo "Writing $out_file"
    rosmsg info $item > $out_file
done

echo -e "\nDone.\n"
