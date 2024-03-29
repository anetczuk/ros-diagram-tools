#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/codedistribution/generate.sh

$SCRIPT_DIR/catkinlist/generate.sh

$SCRIPT_DIR/rostopiclist/generate.sh

$SCRIPT_DIR/rosnodelist/generate.sh

$SCRIPT_DIR/roslaunch/generate.sh


source $SCRIPT_DIR/../../doc/generate_small.sh
