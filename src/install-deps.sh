#!/bin/bash

set -eu

## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


### required for pygraphviz
#sudo apt install graphviz-dev


# ## ensure required version of pip3
# pip3 install --upgrade 'pip>=18.0'


## install requirements
pip3 install -r $SCRIPT_DIR/requirements.txt


echo -e "\ninstallation done\n"
