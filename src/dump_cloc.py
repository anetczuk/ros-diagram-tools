#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2022 Arkadiusz Netczuk <dev.arnet@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import os
import logging

import re
import json
import subprocess


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
    with open(out_path, 'w', encoding ='utf8') as json_file:
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

    overall = parse_code( output )
    json    = parse_code( output, "JSON" )
    if json < 1:
        return overall
    return overall - json


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


def write_file( file_path, content ):
    with open( file_path, 'w', encoding='utf-8' ) as content_file:
        content_file.write( content )


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='cloc dump')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--cloc_dir', action='store', required=False, default="",
                         help="Directory to analyze by 'cloc'" )
    parser.add_argument( '--out_path', action='store', required=False, default="", help="Output file" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    read_data( args.cloc_dir, args.out_path )


if __name__ == '__main__':
    import argparse

    main()
