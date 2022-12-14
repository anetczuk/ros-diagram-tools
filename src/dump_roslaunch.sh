#!/bin/bash

##
## Dump rostopic info to files
##

##set -eu
set -e


if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters -- expected two parameters: entry launch file and output directory"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

LAUNCH_FILE="$1"
INFO_DIR="$2"

mkdir -p $INFO_DIR


echo "Dumping data to $INFO_DIR"


roslaunch $LAUNCH_FILE --files > "$INFO_DIR/list.txt"

items_list=$(roslaunch $LAUNCH_FILE --files)


for item in $items_list; do
    out_file="$INFO_DIR/"$(echo "$item" | sed "s/\//_/g")".txt"
    echo "Writing $out_file"
    roslaunch $item --nodes > $out_file
done

echo -e "\nDone.\n"
