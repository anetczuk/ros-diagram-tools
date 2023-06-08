#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import re
import json
import subprocess

from showgraph.io import read_file


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_data( cloc_dir, out_path ):
    sub_dirs = next(os.walk( cloc_dir ))[1]
    dirs_list = []
    for subdir in sub_dirs:
        sub_path = os.path.join( cloc_dir, subdir )
        dirs_list.append( sub_path )
    data_dir = read_dirs( dirs_list )
    with open(out_path, 'w', encoding='utf8') as json_file:
        json.dump( data_dir, json_file, allow_nan=False )


def read_dirs( dirs_list ):
    _LOGGER.info( "checking directories: %s", dirs_list )
    ret_dict = {}
    for dir_path in dirs_list:
        dir_name = os.path.basename( dir_path )
        lines = cloc_directory( dir_path )
        if lines < 1:
            continue
        ret_dict[ dir_name ] = lines
    return ret_dict


def cloc_directory( sources_dir ):
    _LOGGER.info( "counting code on: %s", sources_dir )
    if os.path.islink( sources_dir ):
        result = subprocess.run( ["cloc", "--sum-one", "--follow-links", sources_dir], capture_output=True, check=True )
    else:
        result = subprocess.run( ["cloc", "--sum-one", sources_dir], capture_output=True, check=True )

    output = result.stdout.decode("utf-8")

    ## _LOGGER.info( "cloc output:\n%s", output )

    overall_code = parse_code( output )
    json_code    = parse_code( output, "JSON" )
    if json_code < 1:
        return overall_code
    return overall_code - json_code


def parse_cloc_file( file_path, language="SUM:", ignore=None ):
    if ignore is None:
        ignore = []
    try:
        content = read_file( file_path )
        language_lines = parse_code( content, language=language )
        ignore_sum = 0
        for ignore_item in ignore:
            ignore_lines = parse_code( content, language=ignore_item )
            if ignore_lines > 0:
                ignore_sum += ignore_lines
        return language_lines - ignore_sum
    except BaseException as exc:
        _LOGGER.error( "error while loading file: %s content:\n%s", file_path, content )
        raise


def parse_code( content, language="SUM:" ):
    for line in content.splitlines():
        if len(line) < 1:
            continue
        if line.startswith( language ):
            ## dependency
            pattern = "^" + language + r"\D*(\d*)\s*(\d*)\s*(\d*)\s*(\d*)\s*$"
            matched = re.findall( pattern, line )
            if len(matched) != 1:
                _LOGGER.warning( "invalid state for line: %s", line )
                continue
            result = matched[0]
            if len(result) != 4:
                _LOGGER.warning( "invalid state for line: %s", line )
                continue
            return int( result[3] )
    return -1


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='parse cloc')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--clocrundir', action='store', required=True, default="",
                         help="Directory to analyze by 'cloc'" )
    parser.add_argument( '--outfile', action='store', required=True, default="", help="Output file" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    read_data( args.clocrundir, args.outfile )


if __name__ == '__main__':
    import argparse

    main()
