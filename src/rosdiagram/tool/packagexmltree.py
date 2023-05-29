# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import argparse

from showgraph.io import read_file, read_list
from showgraph.graphviz import Graph, preserve_neighbour_nodes, set_nodes_style,\
    preserve_top_subgraph

from rosdiagram.graphviztohtml import generate_graph_html


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


def get_items_list( deps_dict ):
    ret_list = []
    ret_list.extend( deps_dict.keys() )
    for _, deps in deps_dict.items():
        ret_list.extend( deps )
    return ret_list


def generate_pkg_graph( deps_dict, node_shape="octagon", 
                        top_items=None, highlight_items=None, preserve_neighbour_items=None, paint_function=None ):
    pkg_graph: Graph = generate_graph( deps_dict, node_shape=node_shape )
    if top_items:
        preserve_top_subgraph( pkg_graph, top_items )
    if preserve_neighbour_items:
        preserve_neighbour_nodes( pkg_graph, preserve_neighbour_items, 0 )
    set_min_max_rank( pkg_graph )
    if paint_function:
        paint_function( pkg_graph )
    paint_nodes( pkg_graph, highlight_items )
    return pkg_graph

    
def generate_graph( deps_dict, node_shape="octagon" ):
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


def paint_nodes( graph: Graph, paint_list ):
    style = { "style": "filled",
              "fillcolor": "yellow"
              }
    set_nodes_style( graph, paint_list, style )


## ===============================================================


def generate( catkin_list_file, node_shape="box",
              top_items=None, highlight_items=None, preserve_neighbour_items=None, paint_function=None ):
    content   = read_file( catkin_list_file )
    data_dict = parse_content( content, build_deps=False )
    graph     = generate_pkg_graph( data_dict, node_shape,
                                    top_items=top_items, highlight_items=highlight_items, preserve_neighbour_items=preserve_neighbour_items, paint_function=paint_function )
    return graph


def generate_pages( deps_dict, out_dir, config_params_dict=None ):
    if config_params_dict is None:
        config_params_dict = {}

    top_list       = config_params_dict.get( "top_list", [] )
    highlight_list = config_params_dict.get( "highlight_list", [] )
    paint_function = config_params_dict.get( "paint_function", None )

    main_graph: Graph = generate_pkg_graph( deps_dict, top_items=top_list, highlight_items=highlight_list, paint_function=paint_function )

    all_items = sorted( main_graph.getNodeNamesAll() )

    params_dict = { "style": {},
                    "labels_dict": {},
                    "main_page": { "graph": main_graph,
                                   "lists": [ { "title": "Graph items", "items": all_items } ]
                                   },
                    "sub_pages": generate_subpages_dict( deps_dict, all_items, highlight_list,
                                                         top_list=top_list, paint_function=paint_function )
                    }

    generate_graph_html( out_dir, params_dict )


def generate_subpages_dict( deps_dict, items_list, highlight_list=None, top_list=None, paint_function=None ):
    if highlight_list is None:
        highlight_list = []

    sub_items = {}
    for item_id in items_list:
        _LOGGER.info( "preparing subpage data for %s", item_id )

        item_dict = {}
        sub_items[ item_id ] = item_dict

        item_graph: Graph = generate_pkg_graph( deps_dict, 
                                                top_items=top_list, highlight_items=highlight_list, preserve_neighbour_items=[item_id], 
                                                paint_function=paint_function )

        item_dict[ "graph" ]       = item_graph
        item_dict[ "msg_type" ]    = ""
        item_dict[ "msg_content" ] = ""

        nodes_list = sorted( list( item_graph.getNodeNamesAll() ) )
        item_dict[ "lists" ] = [ { "title": "Graph items", "items": nodes_list } ]

    return sub_items


## ===================================================================


def configure_parser( parser ):
    parser.description = 'catkin packages graph'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '-f', '--file', action='store', required=False, default="",
                         help="Read 'catkin list' output from file" )
    parser.add_argument( '--nodeshape', action='store', required=False, default=None, help="Shape of node: 'box', 'octagon' or other value supprted by GraphViz dot" )
    parser.add_argument( '--topitems', action='store', required=False, default="", help="File with list of items to filter on top" )
    parser.add_argument( '--highlightitems', action='store', required=False, default="", help="File with list of items to highlight" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help="Output HTML" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args, paint_function=None ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.WARNING )

    node_shape = args.nodeshape
    if node_shape is None:
        node_shape = "octagon"

    content        = read_file( args.file )
    data_dict      = parse_content( content, build_deps=False )
    top_list       = read_list( args.topitems )
    highlight_list = read_list( args.highlightitems )

    _LOGGER.info( "generating packages graph" )
    graph = generate_pkg_graph( data_dict, node_shape, top_items=top_list, highlight_items=highlight_list, paint_function=paint_function )

    if len( args.outraw ) > 0:
        graph.writeRAW( args.outraw )
    if len( args.outpng ) > 0:
        graph.writePNG( args.outpng )

    ##
    ## generate HTML data
    ##
    if args.outhtml and len( args.outdir ) > 0:
        _LOGGER.info( "generating HTML graph" )
        os.makedirs( args.outdir, exist_ok=True )
        config_params_dict = {  "top_list": top_list,
                                "highlight_list": highlight_list,
                                "paint_function": paint_function
                                }
        generate_pages( data_dict, args.outdir, config_params_dict )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
