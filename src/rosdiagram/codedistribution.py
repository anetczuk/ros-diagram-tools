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

# pylint: disable=C0413

import os
import sys
import logging


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


if __name__ == '__main__':
    ## allow having executable script inside package and have proper imports
    ## replace directory of main package (prevent inconsistent imports)
    sys.path[0] = os.path.join( SCRIPT_DIR, os.pardir )


import math
import re
import subprocess

from rosdiagram.graph import Graph


## ===================================================================


def read_dir( sources_dir ):
    sub_dirs = next(os.walk( sources_dir ))[1]
    dirs_list = []
    for subdir in sub_dirs:
        sub_path = os.path.join( sources_dir, subdir )
        dirs_list.append( sub_path )
    return read_dirs( dirs_list )


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
    result = subprocess.run( ["cloc", sources_dir], capture_output=True, check=True )
    output = result.stdout.decode("utf-8")
    return parse_code( output )

#     ret_dict = {}
#     ret_dict[ language ] = parse_code( output, language )
#     return ret_dict


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


def generate_graph( cloc_dict ):
    dot_graph = Graph()
    dot_graph.setEngine( "neato" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
#     base_graph.set_rankdir( 'LR' )

    max_val = -1
    for key, val in cloc_dict.items():
        max_val = max( max_val, val )
    if max_val < 1:
        return dot_graph
    
    MAX_SIZE  = 8
    width_dict = {}
    for key, val in cloc_dict.items():
        #### make circles areas proportional
        ## val / max = PI*r^2 / PI*R^2
        ## val / max = r^2 / R^2
        ## val / max = (2w)^2 / (2W)^2
        ## val / max = 4*w^2 / 4*W^2
        ## val / max = w^2 / W^2
        ## val / max = (w/W)^2
        ## sqrt( val / max ) = w / W
        ## w = sqrt( val / max ) * W
        factor = float( val ) / max_val
        new_val = math.sqrt( factor ) * MAX_SIZE
        width_dict[ key ] = new_val

    ## generate main graph
    for name, lines_num in cloc_dict.items():
        node  = dot_graph.addNode( f"{name}\n{lines_num}", shape="circle" )
        width = width_dict[ name ]
        node.set( "width", width )
        node.set( "fixedsize", "true" )

    return dot_graph


# def set_min_max_rank( dot_graph: Graph ):
#     ## set nodes rank
#     bottom_nodes = dot_graph.getNodesBottom()
#     # print( "bottom:", get_nodes_names( bottom_nodes ) )
#     dot_graph.setNodesRank( bottom_nodes, "max" )
#
#     top_nodes = dot_graph.getNodesTop()
#     # print( "top:", get_nodes_names( top_nodes ) )
#     dot_graph.setNodesRank( top_nodes, "min" )


def generate( sources_dir ):
    data_dict = read_dir( sources_dir )
    graph     = generate_graph( data_dict )
    return graph


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='code distribution')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--dir', action='store', required=False, default="",
                         help="Directory to analyze by 'cloc'" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
#     parser.add_argument( '--filter', action='store', required=False, default="",
#                          help="Filter packages with items in file" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    graph = generate( args.dir )

    if len( args.outraw ) > 0:
        graph.writeRAW( args.outraw )
    if len( args.outpng ) > 0:
        graph.writePNG( args.outpng )


if __name__ == '__main__':
    import argparse

    main()
