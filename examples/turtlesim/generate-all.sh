#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/catkinschedule/generate.sh

$SCRIPT_DIR/nodegraph/generate.sh

$SCRIPT_DIR/rosbag/generate.sh
