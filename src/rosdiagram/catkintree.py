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
import sys
import logging


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


if __name__ == '__main__':
    ## allow having executable script inside package and have proper imports
    ## replace directory of main package (prevent inconsistent imports)
    sys.path[0] = os.path.join( SCRIPT_DIR, os.pardir )


from rosdiagram.graph import Graph
from rosdiagram.io import read_file


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


def generate_graph( deps_dict ):
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## generate main graph
    for key, vals in deps_dict.items():
        dot_graph.addNode( key, shape="box" )
        for dep in vals:
            dot_graph.addNode( dep, shape="box" )
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


def generate( catkin_list_file ):
    content   = read_file( catkin_list_file )
    data_dict = parse_content( content, build_deps=False )
    graph     = generate_graph( data_dict )
    set_min_max_rank( graph )
    return graph


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='catkin packages tree')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '-f', '--file', action='store', required=False, default="",
                         help="Read 'catkin list' output from file" )
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

    graph = generate( args.file )

    if len( args.outraw ) > 0:
        graph.writeRAW( args.outraw )
    if len( args.outpng ) > 0:
        graph.writePNG( args.outpng )


if __name__ == '__main__':
    import argparse

    main()
