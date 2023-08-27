# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse
import json

from showgraph.io import read_list, prepare_filesystem_name, write_file


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_pack( pack_list_path ):
    ret_list = []
    pack_list = read_list( pack_list_path )
    for item in pack_list:
        data = item.split(" ")
        ret_list.append( data )
    return ret_list


def read_launch( launch_dir ):
    nodes_dict = {}
    launch_path = os.path.join( launch_dir, "list.txt" )
    launch_list = read_list( launch_path )
    for item in launch_list:
        launch_filename  = prepare_filesystem_name( item )
        launch_item_path = os.path.join( launch_dir, launch_filename + ".txt" )
        nodes_list = read_list( launch_item_path )
        nodes_dict[ item ] = nodes_list
    return nodes_dict


def classify_nodes( packdumppath, launchdumppath ):
    pack_list = read_pack( packdumppath )
    if len(pack_list) < 1:
        _LOGGER.warning( "no data found in %s", packdumppath )
        return None

    launch_dict = read_launch( launchdumppath )
    if len(launch_dict) < 1:
        _LOGGER.warning( "no data found in %s", launchdumppath )
        return None

    pack_nodes_dict = {}
    for pack in pack_list:
        pack_name = pack[0]
        pack_dir  = pack[1]

        nodes_list = []
        for launch_path, launch_nodes in launch_dict.items():
            if pack_dir in launch_path:
                nodes_list.extend( launch_nodes )

        if nodes_list:
            #print( "found nodes:", pack_name, pack_dir, nodes_list )
            pack_nodes_dict[ pack_name ] = { "path": pack_dir,
                                             "nodes": nodes_list,
                                             }

    return pack_nodes_dict


## ===================================================================


def configure_parser( parser ):
    parser.description = 'match nodes to packages'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--packdumppath', action='store', required=True, default="",
                         help="Path to file containing dumped 'rospack' output" )
    parser.add_argument( '--launchdumppath', action='store', required=True, default="",
                         help="Path fo directory containing dumped 'roslaunch' output" )
    parser.add_argument( '--outfile', action='store', required=False, default="", help="Path to output file" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    pack_nodes_dict = classify_nodes( args.packdumppath, args.launchdumppath )
    if not pack_nodes_dict:
        return

    if args.outfile:
        # content = str( pack_nodes_dict )
        content = json.dumps( pack_nodes_dict, indent=4 )
        write_file( args.outfile, content )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
