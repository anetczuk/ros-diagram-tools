#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


$SCRIPT_DIR/catkinlist/generate.sh

$SCRIPT_DIR/rostopiclist/generate.sh

$SCRIPT_DIR/codedistribution/generate.sh
