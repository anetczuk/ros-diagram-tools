#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


dot -Tpng "$SCRIPT_DIR/tools_graph.gv.txt" -o "$SCRIPT_DIR/tools_graph.png"
