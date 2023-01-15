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


$SCRIPT_DIR/dump_rospack.sh "$INFO_DIR/packinfo"
$SCRIPT_DIR/dump_rosnode.sh "$INFO_DIR/nodeinfo"
$SCRIPT_DIR/dump_rostopic.sh "$INFO_DIR/topicinfo"
$SCRIPT_DIR/dump_rosservice.sh "$INFO_DIR/serviceinfo"
$SCRIPT_DIR/dump_rossrv.sh "$INFO_DIR/srvinfo"
$SCRIPT_DIR/dump_rosmsg.sh "$INFO_DIR/msginfo"


echo -e "\nDone."
