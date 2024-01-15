#!/bin/bash


##
## Generate diagrams based on data in local 'dump' dir
##


set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/generate.sh $SCRIPT_DIR


source $SCRIPT_DIR/../../doc/generate_small.sh
