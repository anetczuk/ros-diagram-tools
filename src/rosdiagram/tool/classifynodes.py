# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import re
from typing import List
import argparse

from rosdiagram.htmlgenerator import generate_graph_html
from rosdiagram.graphviz import Graph, unquote_name_list, set_node_labels
from rosdiagram.io import read_list, prepare_filesystem_name, read_file,\
    write_file
from rosdiagram.utils import get_create_item
import json


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


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='classify nodes')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--pack_list_file', action='store', required=True, default="",
                         help="Dump file containing 'rospack' output" )
    parser.add_argument( '--launch_dump_dir', action='store', required=True, default="",
                         help="Dump directory containing 'roslaunch' output data" )
    parser.add_argument( '--out_file', action='store', required=False, default="", help="Output map file" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    pack_list = read_pack( args.pack_list_file )
    if len(pack_list) < 1:
        _LOGGER.warning( "no data found in %s", args.pack_list_file )
        return

    launch_dict = read_launch( args.launch_dump_dir )
    if len(launch_dict) < 1:
        _LOGGER.warning( "no data found in %s", args.launch_dump_dir )
        return

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

    if args.out_file:
        # content = str( pack_nodes_dict )
        content = json.dumps( pack_nodes_dict, indent=4 )
        write_file( args.out_file, content )
