#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


TOOL_DIR="$SCRIPT_DIR/../../../src"
CHECK_DIR="$TOOL_DIR"

DUMP_DIR="$SCRIPT_DIR/dump"

rm -rf "$DUMP_DIR"

mkdir -p $DUMP_DIR


$TOOL_DIR/dump_scripts/rosdumptools.py roslaunch --launchfile "$SCRIPT_DIR/launch/launch_master.launch" \
                                                 --outdir $DUMP_DIR $@

#$TOOL_DIR/rosdiagramdump.py dumproslaunch --launchfile "$SCRIPT_DIR/launch/launch_master.launch" \
#                                          --outdir $DUMP_DIR $@
