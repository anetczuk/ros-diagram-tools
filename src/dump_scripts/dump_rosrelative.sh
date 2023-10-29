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
    echo "Illegal number of parameters -- expected one parameter: {output directory}"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

INFO_DIR="$1"

mkdir -p $INFO_DIR


## $SCRIPT_DIR/dump_catkindeps.sh runs on catkin workspace not on running roscore


## $SCRIPT_DIR/dump_clocdir.sh runs on source directory


echo "executing: $SCRIPT_DIR/dump_rospack.sh $INFO_DIR/packinfo"
if [[ $* == *--fast* ]]; then
    $SCRIPT_DIR/rosdumptools.py rospack --outdir "$INFO_DIR/packinfo"
else
    $SCRIPT_DIR/dump_rospack.sh "$INFO_DIR/packinfo"
fi


echo "executing: $SCRIPT_DIR/dump_clocpack.sh $INFO_DIR/clocpackinfo"
$SCRIPT_DIR/dump_clocpack.sh "$INFO_DIR/packinfo/list.txt" "$INFO_DIR/clocpackinfo"


## $SCRIPT_DIR/dump_roslaunch.sh requires to pass launch file


echo "executing: $SCRIPT_DIR/dump_rosparam.sh $INFO_DIR/paraminfo"
$SCRIPT_DIR/dump_rosparam.sh "$INFO_DIR/paraminfo"


echo "executing: $SCRIPT_DIR/dump_rosnode.sh $INFO_DIR/nodeinfo"
if [[ $* == *--fast* ]]; then
    $SCRIPT_DIR/rosdumptools.py rosnode --outdir "$INFO_DIR/nodeinfo"
else
    $SCRIPT_DIR/dump_rosnode.sh "$INFO_DIR/nodeinfo"
fi


echo "executing: $SCRIPT_DIR/dump_rostopic.sh $INFO_DIR/topicinfo"
if [[ $* == *--fast* ]]; then
    $SCRIPT_DIR/rosdumptools.py rostopic --outdir "$INFO_DIR/topicinfo"
else
    $SCRIPT_DIR/dump_rostopic.sh "$INFO_DIR/topicinfo"
fi


echo "executing: $SCRIPT_DIR/dump_rosservice.sh $INFO_DIR/serviceinfo"
if [[ $* == *--fast* ]]; then
    $SCRIPT_DIR/rosdumptools.py rosservice --outdir "$INFO_DIR/serviceinfo"
else
    $SCRIPT_DIR/dump_rosservice.sh "$INFO_DIR/serviceinfo"
fi


echo "executing: $SCRIPT_DIR/dump_rosmsg.sh $INFO_DIR/msginfo"
$SCRIPT_DIR/rosparsetools.py extractmsgs --topicsdumpdir "$INFO_DIR/topicinfo" --outlist "$INFO_DIR/msginfo/list.txt"
if [[ $* == *--fast* ]]; then
    $SCRIPT_DIR/rosdumptools.py rosmsg --outdir "$INFO_DIR/msginfo" --listprovided
else
    $SCRIPT_DIR/dump_rosmsg.sh "$INFO_DIR/msginfo" --listprovided
fi


echo "executing: $SCRIPT_DIR/dump_rossrv.sh $INFO_DIR/srvinfo"
$SCRIPT_DIR/rosparsetools.py extractsrvs --servicesdumpdir "$INFO_DIR/serviceinfo" --outlist "$INFO_DIR/srvinfo/list.txt"
if [[ $* == *--fast* ]]; then
    $SCRIPT_DIR/rosdumptools.py rossrv --outdir "$INFO_DIR/srvinfo" --listprovided
else
    $SCRIPT_DIR/dump_rossrv.sh "$INFO_DIR/srvinfo" --listprovided
fi


echo -e "\nDone."
