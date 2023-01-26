#!/bin/bash

#set -eu
set -u


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")


src_dir=$SCRIPT_DIR/../src


## E115 intend of comment
## E126 continuation line over-indented for hanging indent
## E201 whitespace after '('
## E202 whitespace before ')'
## E221 multiple spaces before equal operator
## E241 multiple spaces after ':'
## E262 inline comment should start with '# '
## E265 block comment should start with '# '
## E266 too many leading '#' for block comment
## E402 module level import not at top of file
## E501 line too long (80 > 79 characters)
## W391 blank line at end of file
## D    all docstyle checks
ignore_errors=E115,E126,E201,E202,E221,E241,E262,E265,E266,E402,E501,W391,D


echo "running pycodestyle"
pycodestyle --show-source --statistics --count --ignore=$ignore_errors $src_dir
exit_code=$?

if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi

echo "pycodestyle -- no warnings found"


## F401 'PyQt5.QtCore' imported but unused
ignore_errors=$ignore_errors,F401


echo "running flake8"
python3 -m flake8 --show-source --statistics --count --ignore=$ignore_errors $src_dir
exit_code=$?

if [ $exit_code -ne 0 ]; then
    echo -e "\nflake8 errors found"
    exit $exit_code
fi

echo "flake8 -- no warnings found"


modules_paths=()
for dir in $src_dir/*/; do
    if [ -f "$dir/__init__.py" ]; then
        modules_paths+=( "$dir" )
    fi
done

echo "running pylint3"
echo "to ignore warning for module put following line on top of file: # pylint: disable=<check_id>"
echo "to ignore warning for one line put following comment in end of line: # pylint: disable=<check_id>"
pylint --rcfile=$SCRIPT_DIR/pylint3.config ${modules_paths[@]} $src_dir/*.py
exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi
echo "pylint3 -- no warnings found"
