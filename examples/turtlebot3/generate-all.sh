#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/generate.sh $SCRIPT_DIR


$SCRIPT_DIR/../generate_small.sh
