# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import argparse

from showgraph.io import read_file, read_list, read_dict, prepare_filesystem_name
from showgraph.graphviz import Graph, preserve_neighbour_nodes, set_nodes_style,\
    preserve_top_subgraph

from rosdiagram.graphviztohtml import generate_from_template, set_node_graph_ranks,\
    DEFAULT_ACTIVE_NODE_STYLE, convert_links_list, set_node_html_attribs


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


# if 'build_deps' is set to True then use build dependency, otherwise use run dependency
def parse_catkin_content( catkin_deps_path, build_deps=True ):
    content = read_file( catkin_deps_path )

    deps_dict = {}

    package = None
    build_packages = True
    for line in content.splitlines():
        if len(line) < 1:
            continue

        if line[0] != ' ':
            ## new package
            package = line[:-1]
            deps_dict[ package ] = {"path": "",
                                    "deps": []
                                    }
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
            pkg_data = deps_dict[ package ]
            deps_list = pkg_data.setdefault("deps", [])
            deps_list.append( line[4:] )
            continue

        _LOGGER.warning( "unhandled case: %s", line )

    return deps_dict


def read_pack_data( pack_dump_dir ):
    pack_dict = {}
    launch_path = os.path.join( pack_dump_dir, "list.txt" )
    launch_list = read_list( launch_path )
    for item in launch_list:
        data = item.split(" ")
        package_name = data[0]
        package_path = data[1]
        pack_data_filename  = prepare_filesystem_name( package_name )
        pack_data_path = os.path.join( pack_dump_dir, pack_data_filename + ".txt" )
        deps_list = read_list( pack_data_path )
        pack_dict[ package_name ] = {"path": package_path,
                                     "deps": deps_list              # dependencies of package
                                     }
    return pack_dict


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
    for pkg_id, pkg_data in deps_dict.items():
        dot_graph.addNode( pkg_id, shape=node_shape )
        pkg_deps = pkg_data.get( "deps", [] )
        for dep in pkg_deps:
            dot_graph.addNode( dep, shape=node_shape )
            dot_graph.addEdge( pkg_id, dep )
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
    if not paint_list:
        return
    style = { "style": "filled",
              "fillcolor": "yellow"
              }
    set_nodes_style( graph, paint_list, style )


## ===============================================================


## generate packages graph
def generate( catkin_list_file, node_shape="box",
              top_items=None, highlight_items=None, preserve_neighbour_items=None, paint_function=None ):
    data_dict = parse_catkin_content( catkin_list_file, build_deps=False )
    graph     = generate_pkg_graph( data_dict, node_shape,
                                    top_items=top_items, highlight_items=highlight_items,
                                    preserve_neighbour_items=preserve_neighbour_items, paint_function=paint_function )
    return graph


def generate_pages( deps_dict, out_dir, outhtml, outmarkdown, config_params_dict=None ):
    os.makedirs( out_dir, exist_ok=True )

    if config_params_dict is None:
        config_params_dict = {}

    top_list          = config_params_dict.get( "top_list", [] )
    highlight_list    = config_params_dict.get( "highlight_list", [] )
    nodes_classification = config_params_dict.get( "nodes_classification", {} )
    nodes_description = config_params_dict.get( "nodes_description", {} )
    paint_function    = config_params_dict.get( "paint_function", None )
    main_title        = config_params_dict.get( "main_title", None )

    OUTPUT_NODES_REL_DIR = os.path.join( "nodes" )
    main_graph_name = "full_graph"

    ## generate main page graph
    main_graph: Graph = generate_pkg_graph( deps_dict, top_items=top_list, highlight_items=highlight_list,
                                            paint_function=paint_function )
    main_graph.setName( main_graph_name )
    set_node_html_attribs( main_graph, OUTPUT_NODES_REL_DIR )

    ## generate sub pages
    sub_output_dir = os.path.join( out_dir, OUTPUT_NODES_REL_DIR )
    os.makedirs( sub_output_dir, exist_ok=True )

    all_items = sorted( main_graph.getNodeNamesAll() )          ## graph contains filtered nodes

    item_filename  = prepare_filesystem_name( main_graph_name )
    main_page_link = os.path.join( os.pardir, item_filename + ".autolink" )

    subpages_dict = generate_subpages( sub_output_dir, outhtml, outmarkdown,
                                       deps_dict, all_items, main_page_link,
                                       highlight_list, nodes_classification, nodes_description,
                                       top_list=top_list, paint_function=paint_function )

    ## generate main page
    packages_desc = nodes_description.get( "package", None )
    packages_data_list = convert_links_list( all_items, subpages_dict, OUTPUT_NODES_REL_DIR,
                                             nodes_description=packages_desc )

    main_dict = {   "style": {},
                    "graph": main_graph,
                    #"graph_label": "packages graph",
                    "graph_label": main_graph_name,
                    "graph_packages": packages_data_list
                    }
    if main_title is not None:
        main_dict[ "main_title" ] = main_title

    if outhtml:
        template = "packagetree.html"
        generate_from_template( out_dir, main_dict, template_name=template )

    if outmarkdown:
        template = "packagetree.md"
        generate_from_template( out_dir, main_dict, template_name=template )


## returns dict: { <item_id>: <item_data_dict> }
def generate_subpages( sub_output_dir, outhtml, outmarkdown,                    # pylint: disable=R0913
                       deps_dict, sub_items_list, main_page_link,
                       highlight_list=None, nodes_classification=None, nodes_description=None,
                       top_list=None, paint_function=None ):
    if highlight_list is None:
        highlight_list = []
    if nodes_classification is None:
        nodes_classification = {}
    if nodes_description is None:
        nodes_description = {}

    packages_desc = nodes_description.get( "package", None )

    subpages_dict = {}
    for item_id in sub_items_list:
        _LOGGER.info( "preparing subpage data for %s", item_id )

        item_dict = {}
        subpages_dict[ item_id ] = item_dict

        item_graph: Graph = generate_pkg_graph( deps_dict,
                                                top_items=top_list, highlight_items=highlight_list,
                                                preserve_neighbour_items=[item_id],
                                                paint_function=paint_function )

        set_node_graph_ranks( item_graph, item_id )
        set_nodes_style( item_graph, [item_id], style_dict=DEFAULT_ACTIVE_NODE_STYLE )
        set_node_html_attribs( item_graph, "" )

        pkg_classify_data = nodes_classification.get( item_id, {} )

        pkg_data = deps_dict.get( item_id, {} )
        pkg_path = pkg_data.get( "path", "" )
        if not pkg_path:
            pkg_path = pkg_classify_data.get("path", "")

        item_dict[ "graph" ]          = item_graph
        #item_dict[ "graph_label" ]    = f"package '{item_id}' graph"
        item_dict[ "graph_label" ]    = item_graph.getName()
        item_dict[ "main_page_link" ] = main_page_link
        item_dict[ "pkg_path" ]       = pkg_path
        item_dict[ "pkg_nodes" ]      = pkg_classify_data.get("nodes", [])

        nodes_list     = sorted( list( item_graph.getNodeNamesAll() ) )
        converted_list = convert_links_list( nodes_list, sub_items_list, "", nodes_description=packages_desc )
        item_dict[ "graph_packages" ] = converted_list

        _LOGGER.info( "preparing page for item %s", item_id )

        if outhtml:
            template = "packagetree.html"
            generate_from_template( sub_output_dir, item_dict, template_name=template )

        if outmarkdown:
            template = "packagetree.md"
            generate_from_template( sub_output_dir, item_dict, template_name=template )

    return subpages_dict


## ===================================================================


def configure_parser( parser ):
    parser.description = 'Packages graph. Tool can be feed with catkin putput (based on package.xml) ' \
                         'or with rospack output.'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--catkinlistfile', action='store', required=False, default="",
                         help="Read 'catkin list' data from file" )
    parser.add_argument( '--packdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rospack' output" )
    parser.add_argument( '--classifynodesfile', action='store', required=False, default="",
                         help="Nodes classification input file" )
    parser.add_argument( '--nodeshape', action='store', required=False, default=None, help="Shape of node: 'box', 'octagon' or other value supprted by GraphViz dot" )
    parser.add_argument( '--topitems', action='store', required=False, default="", help="File with list of items to filter on top" )
    parser.add_argument( '--highlightitems', action='store', required=False, default="", help="File with list of items to highlight" )
    parser.add_argument( '--descriptionjson', action='store', required=False, default="", help="Path to JSON file with items description" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help='Output HTML' )
    parser.add_argument( '--outmarkdown', action='store_true', help='Output Markdown' )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output directory" )


def process_arguments( args, paint_function=None ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.WARNING )

    data_dict = {}
    if args.catkinlistfile:
        data_dict = parse_catkin_content( args.catkinlistfile, build_deps=False )
    else:
        data_dict = read_pack_data( args.packdumppath )

    nodes_classify_dict = read_dict( args.classifynodesfile )

    top_list       = read_list( args.topitems )
    highlight_list = read_list( args.highlightitems )

    description_dict = read_dict( args.descriptionjson )

    if args.outraw or args.outraw:
        node_shape = args.nodeshape
        if node_shape is None:
            node_shape = "octagon"

        _LOGGER.info( "generating packages graph" )
        graph = generate_pkg_graph( data_dict, node_shape, top_items=top_list,
                                    highlight_items=highlight_list, paint_function=paint_function )

        if args.outraw:
            graph.writeRAW( args.outraw )
        if args.outpng:
            graph.writePNG( args.outpng )

    ##
    ## generate data
    ##
    if (args.outhtml or args.outmarkdown) and args.outdir:
        _LOGGER.info( "generating graphs" )
        config_params_dict = {  "top_list": top_list,
                                "highlight_list": highlight_list,
                                "nodes_classification": nodes_classify_dict,
                                "nodes_description": description_dict,
                                "paint_function": paint_function
                                }
        generate_pages( data_dict, args.outdir, args.outhtml, args.outmarkdown, config_params_dict )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
