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

from typing import Set, List, Any

from showgraph.io import prepare_filesystem_name
from showgraph.graphviz import Graph, set_nodes_style

from rosdiagram.graphviztohtml import generate_from_template, \
    DEFAULT_ACTIVE_NODE_STYLE, prepare_item_link, set_node_html_attribs, convert_links_list
from rosdiagram.tool.rosparamlist import convert_to_html, prepare_params_list


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


OUTPUT_LAUNCHES_REL_DIR = os.path.join( "launches" )


## ===================================================================


def generate_full_graph( launch_dict ) -> Graph:
    dot_graph = Graph()
    dot_graph.setName( "launch_graph" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    launch_deps_dict = get_launch_files_deps(launch_dict)

    ## add nodes
    for launch_item, sub_files in launch_deps_dict.items():
        node_obj = dot_graph.addNode( launch_item, shape="box" )
        if node_obj:
            node_label = os.path.basename(launch_item)
            node_obj.set( "label", node_label )

        for sub_item in sub_files:
            if sub_item == launch_item:
                continue
            sub_obj = dot_graph.addNode( sub_item, shape="box" )
            if sub_obj:
                sub_label = os.path.basename(sub_item)
                sub_obj.set( "label", sub_label )
            print("adding edge:", launch_item, sub_item)
            dot_graph.addEdge( launch_item, sub_item )

    return dot_graph


def get_launch_files_deps(launch_dict):
    launch_deps = {}

    for launch_item in launch_dict:
        sub_files = get_deps(launch_dict, launch_item)

        sub_deps = get_all_deps(launch_dict, sub_files)
        sub_files = sub_files - sub_deps
        if launch_item in sub_files:
            sub_files.remove(launch_item)

        launch_deps[launch_item] = sub_files

    return launch_deps


class LaunchDeps:
    def __init__(self):
        self.visited = set()

    def getAllDeps(self, launch_dict, launch_list):
        self.visited.clear()
        ret_set = set()
        for launch_item in launch_list:
            if launch_item in self.visited:
                continue
            self.visited.add(launch_item)

            sub_files = get_deps(launch_dict, launch_item)
            all_deps = self.getAllDeps(launch_dict, sub_files)
            ret_set.update(sub_files)
            ret_set.update(all_deps)
        return ret_set


def get_all_deps(launch_dict, launch_list):
    deps_obj = LaunchDeps()
    return deps_obj.getAllDeps(launch_dict, launch_list)


def get_deps(launch_dict, launch_item) -> Set[str]:
    launch_data = launch_dict[launch_item]
    launch_config = launch_data.get("config", {})
    sub_files = launch_config.get("roslaunch_files", [])
    sub_files = set(sub_files)
    sub_files.remove(launch_item)
    return sub_files


## =====================================================


def read_launch_data(launch_dump_file):
    with open( launch_dump_file, 'r', encoding='utf-8' ) as content_file:
        return json.load(content_file)


def generate_launch_pages( launch_output_dir, launch_dict, main_graph: Graph, outhtml=True, outmarkdown=False ):
    os.makedirs( launch_output_dir, exist_ok=True )
    main_graph_name = "full_graph"

    _LOGGER.info( "generating main graph" )

    if main_graph is None:
        main_graph = generate_full_graph(launch_dict)
    main_graph.setName( main_graph_name )
    set_node_html_attribs( main_graph, OUTPUT_LAUNCHES_REL_DIR )

    ## generate sup pages
    sub_launch_output_dir = os.path.join( launch_output_dir, OUTPUT_LAUNCHES_REL_DIR )
    os.makedirs( sub_launch_output_dir, exist_ok=True )

    item_filename  = prepare_filesystem_name( main_graph_name )
    main_page_link = os.path.join( os.pardir, item_filename + ".autolink" )

    _LOGGER.info( "generating subpages" )
    generate_subpages( sub_launch_output_dir, launch_dict, main_page_link, outhtml, outmarkdown )

    launch_files = list( launch_dict.keys() )
    launch_data_list = get_launch_files_list( launch_files, OUTPUT_LAUNCHES_REL_DIR )

    nodes_data_list: List[Any] = []
    nodes_added: Set[str] = set()
    for launch_item in launch_files:
        launch_data = launch_dict[launch_item]
        config_data = launch_data["config"]
        data_list = get_config_nodes_list(config_data, OUTPUT_LAUNCHES_REL_DIR, nodes_added)
        nodes_data_list.extend( data_list )
    nodes_data_list.sort()

    _LOGGER.info( "generating main page" )
    main_dict = {   "style": {},
                    "graph": main_graph,
                    "launch_list": launch_data_list,
                    "nodes_list": nodes_data_list
                    }

    if outhtml:
        template = "roslaunchgraph/launchgraph_main.html"
        generate_from_template( launch_output_dir, main_dict, template_name=template )

    if outmarkdown:
        template = "roslaunchgraph/launchgraph_main.md"
        generate_from_template( launch_output_dir, main_dict, template_name=template )


## returns dict: { <item_id>: <item_data_dict> }
def generate_subpages( sub_output_dir, launch_dict, main_page_link, outhtml, outmarkdown ):
    for launch_file, launch_data in launch_dict.items():
        _LOGGER.info( "preparing page for item %s", launch_file )
        config_data = launch_data["config"]

        launch_files = config_data["roslaunch_files"]
        launch_files = set(launch_files)
        launch_files.remove(launch_file)
        launch_files = list(launch_files)
        launch_list = get_launch_files_list(launch_files)

        params_data = config_data["params"]

        fatten_list = []
        for param_data in params_data.values():
            par_key = param_data["key"]
            par_val = param_data["value"]
            fatten_list.append( (par_key, par_val) )
        fatten_list.sort( key=lambda item: item[0], reverse=False )
        params_list = prepare_params_list(fatten_list, sub_output_dir, "")

        nodes_list = config_data["nodes"]
        for node_item in nodes_list:
            remap_args = node_item["remap_args"]
            remap_args = convert_to_html(remap_args)
            node_item["remap_args"] = remap_args

        graph: Graph = generate_full_graph(launch_dict)
        set_node_html_attribs( graph, "" )
        set_nodes_style( graph, [launch_file], style_dict=DEFAULT_ACTIVE_NODE_STYLE )
        item_out_path = prepare_filesystem_name( launch_file )
        graph.setName(item_out_path)

        item_dict = { "launch_file": launch_file,
                      "main_page_link": main_page_link,
                      "graph": graph,
                      "launch_list": launch_list,
                      "nodes_list": nodes_list,
                      "params_list": params_list
                      }

        if outhtml:
            template_item = "roslaunchgraph/launchgraph_launch.html"
            generate_from_template( sub_output_dir, item_dict, template_name=template_item )

        if outmarkdown:
            template_item = "roslaunchgraph/launchgraph_launch.md"
            generate_from_template( sub_output_dir, item_dict, template_name=template_item )


def get_launch_files_list(launch_files, out_subdir=""):
    launch_files.sort()
    launch_data_list = convert_links_list( launch_files, launch_files, out_subdir )
    return launch_data_list


def get_config_nodes_list(config_data, out_subdir, nodes_added=None):
    nodes_data_list = []
    nodes_list = config_data["nodes"]
    for node_dict in nodes_list:
        node_name = node_dict["name"]

        if nodes_added is not None:
            if node_name in nodes_added:
                continue
            nodes_added.add(node_name)

        node_filename = node_dict["filename"]
        node_data = prepare_item_link( node_filename, node_name, True, out_subdir )
        nodes_data_list.append( node_data )
    nodes_data_list.sort()
    return nodes_data_list


## ===================================================================


def configure_parser( parser ):
    parser.description = 'roslaunch connection graph'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--launchfile', action='store', required=False, default="",
                         help="Path to file containing dumped JSON data" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help="Output HTML" )
    parser.add_argument( '--outmarkdown', action='store_true', help='Output Markdown' )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output directory" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    launch_dump_file = args.launchfile
    launch_output_dir = args.outdir
    launch_dict = read_launch_data(launch_dump_file)

    main_graph: Graph = generate_full_graph(launch_dict)
    if args.outraw:
        main_graph.writeRAW( args.outraw )
    if args.outpng:
        main_graph.writePNG( args.outpng )

    ##
    ## generate data
    ##
    outhtml = args.outhtml
    outmarkdown = args.outmarkdown
    if (outhtml or outmarkdown) and launch_output_dir:
        generate_launch_pages( launch_output_dir, launch_dict, main_graph, outhtml=outhtml, outmarkdown=outmarkdown )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
