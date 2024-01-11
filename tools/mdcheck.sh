#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


cd $SCRIPT_DIR

echo -e "\nchecking links in MD files"

python3 -m mdlinkscheck --implicit-heading-id-github -d .. --excludes ".*/tmp/.*"
