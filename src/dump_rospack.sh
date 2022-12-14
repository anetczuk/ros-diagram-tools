#!/bin/bash

##
## Dump rostopic info to files
##

##set -eu
set -e


if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters -- expected one parameter (output directory)"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

INFO_DIR="$1"

mkdir -p $INFO_DIR


echo "Dumping data to $INFO_DIR"


rospack list > "$INFO_DIR/list.txt"

echo -e "\nDone.\n"


#items_list=$(rospack list-names)
#
#
#for item in $items_list; do
#    out_file="$INFO_DIR/"$(echo "$item" | sed "s/\//_/g")".txt"
#    echo "Writing $out_file"
#    rospack info $item > $out_file
#done
#
#echo -e "\nDone.\n"
