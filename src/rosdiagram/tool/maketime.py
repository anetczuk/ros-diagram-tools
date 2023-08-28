# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import argparse
import re
import json
from datetime import datetime

from showgraph.io import read_file, write_file


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


class BuildLog:

    def __init__(self):
        self.object_queue = []
        self._curr_object_name: str = None
        self._curr_object_time: datetime = None        # start time

    def addObjectStart(self, name, start_time):
        self.addObjectEnd(start_time)
        # start of new object
        self._curr_object_name = name
        self._curr_object_time = start_time

    def addObjectEnd(self, end_time):
        if self._curr_object_name is None:
            # no object
            return
        diff = end_time - self._curr_object_time
        diff_secs = diff.total_seconds()
        self.object_queue.append( (self._curr_object_name, diff_secs) )
        self._curr_object_name = None
        self._curr_object_time = None

    def addObject(self, name, start_time, end_time):
        self.addObjectStart( name, start_time )
        self.addObjectEnd( end_time )

    def addLinkingStart(self, start_time):
        self.addObjectEnd(start_time)

    def addTargetFinish(self, _, end_time):
        self.addObjectEnd(end_time)


# make -j1 | ts '[%H:%M:%.S]'
def read_compile_log( log_path ):
    content = read_file( log_path )
    if content is None:
        _LOGGER.warning( "unable to read content from file '%s'", log_path )
        return None

    build_log = BuildLog()

    first_time = None
    last_time = None

    for line in content.splitlines():
        line = line.strip()

        line_time = get_build_timestamp( line )
        if line_time:
            if not first_time:
                first_time = line_time
            last_time = line_time

        object_name = get_after( line, r"Building \S+ object " )
        if object_name:
            build_log.addObjectStart(object_name, line_time)
            continue

        linking_name = get_after( line, "Linking " )
        if linking_name:
            build_log.addLinkingStart(line_time)
            continue

        target_name = get_after( line, "Built target " )
        if target_name:
            build_log.addTargetFinish(target_name, line_time)
            continue

        _LOGGER.warning("unknown entry: %s", line)

    build_log.addObject( "make_time", first_time, last_time )

    return build_log.object_queue


def get_after( content, substring ):
    found = re.search( fr"({substring})(.*)$", content )
    if not found:
        # not found
        return None
    found_groups = found.groups()
    if not found_groups:
        # not found
        return None
    last_group = found_groups[-1]
    return last_group


def get_build_timestamp( content ):
    time_list = re.findall( r"\[(.*?)\] \[", content )
    if len(time_list) != 1:
        return None

    time_text = time_list[0]
    try:
        return datetime.strptime(time_text, "%H:%M:%S.%f")
    except ValueError:
        # content does not match format
        pass
    try:
        return datetime.strptime(time_text, "%Y-%m-%d %H:%M:%S.%f")
    except ValueError:
        # content does not match format
        pass

    raise RuntimeError( f"unable to get time form content: {time_text}" )


## ===================================================================


def configure_parser( parser ):
    parser.description = 'objects compilation time based on make output'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '-clf', '--compilelogfile', action='store', required=True,
                         help="Path to make compile log file" )
    parser.add_argument( '--outfile', action='store', required=False, default="", help="Path to output file" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    compile_list = read_compile_log( args.compilelogfile )
    compile_list.sort( key=lambda item: item[1], reverse=True )
    # pprint.pprint( compile_list, width=999 )

    content = json.dumps( compile_list, indent=4 )
    if args.outfile:
        write_file( args.outfile, content )
    else:
        print( content )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
