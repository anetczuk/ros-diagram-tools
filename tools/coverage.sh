#!/bin/bash

set -eu


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

SOURCE_DIR=$SCRIPT_DIR/../src


timestamp=$(date +%s)
tmpdir=$(dirname $(mktemp -u))

revcrctmp_dir="$tmpdir/revcrc"
htmlcov_dir="$revcrctmp_dir/htmlcov.${timestamp}"
mkdir -p $htmlcov_dir


#coverage_file=$(mktemp ${tmpdir}/revcrc.coverage.${timestamp}.XXXXXX)


echo "Starting coverage"


coverage run --source $SOURCE_DIR --omit */site-packages/* --branch $@

# coverage report

coverage html -d $htmlcov_dir


echo ""
echo "Coverage HTML output: file://$htmlcov_dir/index.html"


## rm ${coverage_file}
