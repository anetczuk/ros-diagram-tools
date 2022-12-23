# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree. 
#

# pylint: disable=C0413

import os
import sys
import logging
import argparse

from rosdiagram.graphviz import Graph
from rosdiagram.io import read_file


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def parse_content( content, build_deps=True ):
    deps_dict = {}

    package = None
    build_packages = True
    for line in content.splitlines():
        if len(line) < 1:
            continue

        if line[0] != ' ':
            ## new package
            package = line[:-1]
            deps_dict[ package ] = []
            continue

        if package is None:
            continue

        if "build_depend" in line:
            build_packages = True
            continue
        if "run_depend" in line:
            build_packages = False
            continue

        if build_packages != build_deps:
            continue

        if line.startswith( "  - " ):
            ## dependency
            deps_list = deps_dict[ package ]
            deps_list.append( line[4:] )
            continue

        _LOGGER.warning( "unhandled case: %s", line )

    return deps_dict


def generate_graph( deps_dict, node_shape="box" ):
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## generate main graph
    for key, vals in deps_dict.items():
        dot_graph.addNode( key, shape=node_shape )
        for dep in vals:
            dot_graph.addNode( dep, shape=node_shape )
            dot_graph.addEdge( key, dep )
    return dot_graph


def set_min_max_rank( dot_graph: Graph ):
    ## set nodes rank
    bottom_nodes = dot_graph.getNodesBottom()
    # print( "bottom:", get_nodes_names( bottom_nodes ) )
    dot_graph.setNodesRank( bottom_nodes, "max" )

    top_nodes = dot_graph.getNodesTop()
    # print( "top:", get_nodes_names( top_nodes ) )
    dot_graph.setNodesRank( top_nodes, "min" )


def generate( catkin_list_file, node_shape="box" ):
    content   = read_file( catkin_list_file )
    data_dict = parse_content( content, build_deps=False )
    graph     = generate_graph( data_dict, node_shape )
    set_min_max_rank( graph )
    return graph


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='catkin packages graph')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '-f', '--file', action='store', required=False, default="",
                         help="Read 'catkin list' output from file" )
    parser.add_argument( '--node_shape', action='store', required=False, default=None, help="Graph RAW output" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
#     parser.add_argument( '--filter', action='store', required=False, default="",
#                          help="Filter packages with items in file" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.WARNING )

    node_shape = args.node_shape
    if node_shape is None:
        node_shape = "octagon"

    graph = generate( args.file, node_shape )

    if len( args.outraw ) > 0:
        graph.writeRAW( args.outraw )
    if len( args.outpng ) > 0:
        graph.writePNG( args.outpng )
