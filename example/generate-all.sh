#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/simple/generate-all.sh

$SCRIPT_DIR/turtlesim/generate-all.sh


## generate small images
$SCRIPT_DIR/generate_small.sh
