#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/codedistribution/generate.sh

$SCRIPT_DIR/catkinlist/generate.sh

$SCRIPT_DIR/rostopiclist/generate.sh

$SCRIPT_DIR/rosnodelist/generate.sh


## generate small images
$SCRIPT_DIR/generate_small.sh
