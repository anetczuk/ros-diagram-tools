#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


OUT_ROOT_DIR="$SCRIPT_DIR/out"

rm -rf "$OUT_ROOT_DIR"

mkdir -p "$OUT_ROOT_DIR"


$SCRIPT_DIR/simple.py


$SCRIPT_DIR/../generate_small.sh
