#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


SRC_DIR="$SCRIPT_DIR/../src"

HELP_PATH=$SCRIPT_DIR/cmd_args.txt

$SRC_DIR/rosdiagramtools.py --help > ${HELP_PATH}


tools=$($SRC_DIR/rosdiagramtools.py --listtools)

IFS=', ' read -r -a tools_list <<< "$tools"


for item in ${tools_list[@]}; do
    echo $item
    echo -e "\n\n\n\n"  >> ${HELP_PATH}
    $SRC_DIR/rosdiagramtools.py $item --help >> ${HELP_PATH}
done
