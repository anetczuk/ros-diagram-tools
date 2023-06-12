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

##set -eu
set -e


if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters -- expected one parameter: {output directory}"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

INFO_DIR="$1"

mkdir -p $INFO_DIR


echo "executing: $SCRIPT_DIR/dump_rospack.sh $INFO_DIR/packinfo"
$SCRIPT_DIR/dump_rospack.sh "$INFO_DIR/packinfo"

echo "executing: $SCRIPT_DIR/dump_clocpack.sh $INFO_DIR/clocpack"
$SCRIPT_DIR/dump_clocpack.sh "$INFO_DIR/packinfo/list.txt" "$INFO_DIR/clocpack"

echo "executing: $SCRIPT_DIR/dump_rosnode.sh $INFO_DIR/nodeinfo"
$SCRIPT_DIR/dump_rosnode.sh "$INFO_DIR/nodeinfo"

echo "executing: $SCRIPT_DIR/dump_rostopic.sh $INFO_DIR/topicinfo"
$SCRIPT_DIR/dump_rostopic.sh "$INFO_DIR/topicinfo"

echo "executing: $SCRIPT_DIR/dump_rosservice.sh $INFO_DIR/serviceinfo"
$SCRIPT_DIR/dump_rosservice.sh "$INFO_DIR/serviceinfo"

echo "executing: $SCRIPT_DIR/dump_rossrv.sh $INFO_DIR/srvinfo"
$SCRIPT_DIR/dump_rossrv.sh "$INFO_DIR/srvinfo"

echo "executing: $SCRIPT_DIR/dump_rosmsg.sh $INFO_DIR/msginfo"
$SCRIPT_DIR/dump_rosmsg.sh "$INFO_DIR/msginfo"

## $SCRIPT_DIR/dump_roslaunch.sh requires to pass launch file

echo -e "\nDone."