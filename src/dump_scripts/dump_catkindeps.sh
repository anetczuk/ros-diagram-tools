#!/bin/bash
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

##
## Dump catkin package dependencies info to files
##

set -eu
#set -x


if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters -- expected one parameter (output directory)"
    exit 1
fi


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

INFO_DIR="$1"

mkdir -p $INFO_DIR


echo "Dumping data to $INFO_DIR"


## dump list of workspace local packages
catkin list -u > "$INFO_DIR/packages.txt"

sort -o "$INFO_DIR/packages.txt" "$INFO_DIR/packages.txt"

## dump packages and it's direct dependencies
catkin list --deps > "$INFO_DIR/list.txt"

echo -e "\nDone.\n"
