#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/create_ws.sh "turtlebot3"

cd "turtlebot3"

$SCRIPT_DIR/codedistribution.sh
$SCRIPT_DIR/catkintree.sh
$SCRIPT_DIR/catkinschedule.sh
$SCRIPT_DIR/rosverify.sh
