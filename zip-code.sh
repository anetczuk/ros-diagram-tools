#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


OUT_PATH="$SCRIPT_DIR/tmp/ros-diagram-tools-master.zip"


cd $SCRIPT_DIR

zip -r "$OUT_PATH" * -x tmp/**\*

#git archive HEAD -o ${OUT_PATH}
