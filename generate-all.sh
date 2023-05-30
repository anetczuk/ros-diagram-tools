#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/examples/generate-all.sh

$SCRIPT_DIR/doc/generate-all.sh
