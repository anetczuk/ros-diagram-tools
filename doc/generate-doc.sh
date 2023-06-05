#!/bin/bash

set -eu


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


SRC_DIR="$SCRIPT_DIR/../src"


generate_tools_help() {
    HELP_PATH=$SCRIPT_DIR/cmd_args_tools.md
    
    echo "## rosdiagramtools.py --help" > ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    $SRC_DIR/rosdiagramtools.py --help >> ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    
    
    tools=$($SRC_DIR/rosdiagramtools.py --listtools)
    
    IFS=', ' read -r -a tools_list <<< "$tools"
    
    
    for item in ${tools_list[@]}; do
        echo $item
        echo -e "\n\n" >> ${HELP_PATH}
        echo "## rosdiagramtools.py $item --help" >> ${HELP_PATH}
        echo -e "\`\`\`" >> ${HELP_PATH}
        $SRC_DIR/rosdiagramtools.py $item --help >> ${HELP_PATH}
        echo -e "\`\`\`"  >> ${HELP_PATH}
    done
}


generate_dump_help() {
    HELP_PATH=$SCRIPT_DIR/cmd_args_dump.md
    
    echo "## rosdiagramdump.py --help" > ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    $SRC_DIR/rosdiagramdump.py --help >> ${HELP_PATH}
    echo -e "\`\`\`" >> ${HELP_PATH}
    
    
    tools=$($SRC_DIR/rosdiagramdump.py --listtools)
    
    IFS=', ' read -r -a tools_list <<< "$tools"
    
    
    for item in ${tools_list[@]}; do
        echo $item
        echo -e "\n\n" >> ${HELP_PATH}
        echo "## rosdiagramdump.py $item --help" >> ${HELP_PATH}
        echo -e "\`\`\`" >> ${HELP_PATH}
        $SRC_DIR/rosdiagramdump.py $item --help >> ${HELP_PATH}
        echo -e "\`\`\`"  >> ${HELP_PATH}
    done
}


$SCRIPT_DIR/tools_graph/generate.sh

generate_tools_help
generate_dump_help

$SCRIPT_DIR/generate_small.sh
