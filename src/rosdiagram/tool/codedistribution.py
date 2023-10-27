# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import math
import json
import argparse
import shutil

from showgraph.graphviz import Graph, set_nodes_style
from showgraph.io import read_list, prepare_filesystem_name

from rosdiagram.clocparser import parse_cloc_file
from rosdiagram.graphviztohtml import generate_from_template


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_cloc_data( clocjsonpath=None, clocdumpdir=None, filteritemspath=None ):
    data_dict = {}
    if clocjsonpath:
        data_dict = read_json_data( clocjsonpath )
    elif clocdumpdir:
        data_dict = read_dir_data( clocdumpdir )

    filter_list = read_list( filteritemspath )
    if filter_list:
        all_packages = list( data_dict.keys() )
        for item in all_packages:
            if item in filter_list:
                continue
            del data_dict[ item ]

    return data_dict


def read_json_data( cloc_path ):
    with open( cloc_path, 'r', encoding='utf-8' ) as content_file:
        content = content_file.read()
        return json.loads( content )


def read_dir_data( dump_path ):
    if not dump_path:
        return None
    cloc_dict = {}
    cloc_list_path = os.path.join( dump_path, "list.txt" )
    _LOGGER.debug( "reading cloc list file: %s", cloc_list_path )
    if not os.path.isfile( cloc_list_path ):
        raise FileNotFoundError( f"unable to find file: {cloc_list_path}" )
    cloc_list = read_list( cloc_list_path )
    for item in cloc_list:
        node_filename  = prepare_filesystem_name( item )
        node_item_path = os.path.join( dump_path, node_filename + ".txt" )
        total_lines = parse_cloc_file( node_item_path, ignore=["JSON"] )
        if total_lines <= 0:
            _LOGGER.warning( "unable to parse: %s", node_item_path )
            continue
        pkg_name = os.path.basename( item )
        cloc_dict[ pkg_name ] = (total_lines, node_item_path)
    return cloc_dict


## ===================================================================


def generate_graph( cloc_dict ):
    dot_graph = Graph()
    dot_graph.setEngine( "neato" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
#     base_graph.set_rankdir( 'LR' )

    lines_dict = {}
    for key, val in cloc_dict.items():
        if isinstance( val, tuple ):
            lines_dict[key] = val[0]
        else:
            lines_dict[key] = val

    max_val = -1
    for key, val in lines_dict.items():
        max_val = max( max_val, val )
    if max_val < 1:
        return dot_graph

    MAX_SIZE  = 8
    width_dict = {}
    for key, val in lines_dict.items():
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
    for name, lines_num in lines_dict.items():
        node  = dot_graph.addNode( name, shape="circle" )
        node.set( "label", f"{name}\n{lines_num}" )
        #node  = dot_graph.addNode( f"{name}\n{lines_num}", shape="circle" )
        width = width_dict[ name ]
        node.set( "width", width )
        node.set( "fixedsize", "true" )
        node.set( "color", "gray" )

    return dot_graph


def generate_pages( data_dict, cloc_graph, out_dir, outhtml, outmarkdown, highlight_list=None ):
    os.makedirs( out_dir, exist_ok=True )

    if cloc_graph is None:
        cloc_graph = generate_graph( data_dict )
        if highlight_list:
            paint_nodes( cloc_graph, highlight_list )

    if cloc_graph is None:
        _LOGGER.error( "unable to generate pages -- no graph" )
        return

    main_graph_name = "full_graph"
    cloc_graph.setName( main_graph_name )

    out_data_dir = os.path.join( out_dir, "data" )
    os.makedirs( out_data_dir, exist_ok=True )

    packages_list = []
    for name, item_data in data_dict.items():
        if not isinstance( item_data, tuple ):
            packages_list.append( (name, None, item_data) )
            continue

        colc_info_path = item_data[1]
        cloc_info_file = os.path.basename(colc_info_path)
        out_cloc_path = os.path.join( out_data_dir, cloc_info_file )
        shutil.copyfile(colc_info_path, out_cloc_path)
        out_cloc_path = os.path.join( out_data_dir, cloc_info_file )

        cloc_data_path = os.path.join( "data", cloc_info_file )
        packages_list.append( (name, cloc_data_path, item_data[0]) )

    packages_list = sorted( packages_list, key=lambda x: x[0] )

    main_dict = {   "style": {},
                    "graph": cloc_graph,
                    "graph_label": main_graph_name,
                    "packages": packages_list
                    }

    if outhtml:
        template = "cloc.html"
        generate_from_template( out_dir, main_dict, template_name=template )

    if outmarkdown:
        template = "cloc.md"
        generate_from_template( out_dir, main_dict, template_name=template )


def paint_nodes( graph: Graph, paint_list ):
    style = { "style": "filled",
              "fillcolor": "yellow"
              }
    set_nodes_style( graph, paint_list, style )


def generate( cloc_path ):
    data_dict = read_json_data( cloc_path )
    graph     = generate_graph( data_dict )
    return graph


## ===================================================================


def configure_parser( parser ):
    parser.description = 'Source code distribution over packages. ' \
                         'Tool can be feed with JSON or with path to output of dumpclocpack tool.'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--clocjsonpath', action='store', required=False, default="",
                         help="Path to JSON file with dumped 'cloc' results" )
    parser.add_argument( '--clocdumpdir', action='store', required=False, default="",
                         help="Path to directory with dumped 'cloc' results" )
    parser.add_argument( '--filteritems', action='store', required=False, default="", help="File with list of items to filter" )
    parser.add_argument( '--highlight', action='store', required=False, default="",
                         help="List with items to highlight" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help='Output HTML' )
    parser.add_argument( '--outmarkdown', action='store_true', help='Output Markdown' )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output directory" )
#     parser.add_argument( '--filter', action='store', required=False, default="",
#                          help="Filter packages with items in file" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    highlight_list = read_list( args.highlight )

    data_dict = read_cloc_data( args.clocjsonpath, args.clocdumpdir, args.filteritems )

    graph = generate_graph( data_dict )

    if graph is not None:
        paint_nodes( graph, highlight_list )

        if len( args.outraw ) > 0:
            graph.writeRAW( args.outraw )
        if len( args.outpng ) > 0:
            graph.writePNG( args.outpng )

    ##
    ## generate data
    ##
    if (args.outhtml or args.outmarkdown) and args.outdir:
        _LOGGER.info( "generating graphs" )
        generate_pages( data_dict, graph, args.outdir, args.outhtml, args.outmarkdown )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
