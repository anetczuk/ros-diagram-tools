# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse
import hashlib

import json

from typing import Set, List, Any

from showgraph.io import prepare_filesystem_name
from showgraph.graphviz import Graph, set_nodes_style

from rosdiagram.graphviztohtml import generate_from_template, \
    DEFAULT_ACTIVE_NODE_STYLE, prepare_item_link, set_node_html_attribs
from rosdiagram.tool.rosparamlist import convert_to_html, prepare_params_list


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


OUTPUT_LAUNCHES_REL_DIR = os.path.join( "launches" )


## ===================================================================


def generate_full_graph( flat_launch_dict ) -> Graph:
    dot_graph = Graph()
    dot_graph.setName( "launch_graph" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for launch_id, launch_data in flat_launch_dict.items():
        launch_file = launch_data["file"]
        launch_file = os.path.normpath(launch_file)

        node_obj = dot_graph.addNode( launch_id, shape="box" )
        if node_obj:
            node_label = os.path.basename(launch_file)
            node_obj.set( "label", node_label )

        launch_includes = launch_data["included"]
        for include_item in launch_includes:
            sub_id = get_launch_id(include_item)
            sub_file = include_item["file"]
            sub_file = os.path.normpath(sub_file)
            if sub_id == launch_id:
                continue
            sub_obj = dot_graph.addNode( sub_id, shape="box" )
            if sub_obj:
                sub_label = os.path.basename(sub_file)
                sub_obj.set( "label", sub_label )
            dot_graph.addEdge( launch_id, sub_id )

    return dot_graph


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
    launch_data = launch_dict.get(launch_item)
    if not launch_data:
        _LOGGER.warning("unable to find item: %s", launch_item)
        return set()
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

    flat_launch_dict = get_flat_dict(launch_dict)

    if main_graph is None:
        main_graph = generate_full_graph(flat_launch_dict)
    main_graph.setName( main_graph_name )
    set_node_html_attribs( main_graph, OUTPUT_LAUNCHES_REL_DIR )

    ## generate main
    generate_main_page(launch_output_dir, launch_dict, main_graph, outhtml, outmarkdown)

    ## generate sup pages
    sub_launch_output_dir = os.path.join( launch_output_dir, OUTPUT_LAUNCHES_REL_DIR )
    os.makedirs( sub_launch_output_dir, exist_ok=True )

    item_filename  = prepare_filesystem_name( main_graph_name )
    main_page_link = os.path.join( os.pardir, item_filename + ".autolink" )

    _LOGGER.info( "generating subpages" )
    generate_subpages( sub_launch_output_dir, flat_launch_dict, main_page_link, outhtml, outmarkdown )


## generate page with list of all includes and all nodes
def generate_main_page(launch_output_dir, launch_dict, main_graph, outhtml=True, outmarkdown=False):
    flat_launch_dict = get_flat_dict(launch_dict)

    launch_data_list = []
    for launch_id, launch_data in flat_launch_dict.items():
        launch_file = launch_data["file"]
        launch_file = os.path.normpath(launch_file)
        item_link = prepare_item_link( launch_id, launch_file, True, OUTPUT_LAUNCHES_REL_DIR )
        launch_data_list.append(item_link)
    launch_data_list.sort()

    nodes_data_list: List[Any] = get_nodes_files_deps(launch_dict, OUTPUT_LAUNCHES_REL_DIR)

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
def generate_subpages( sub_output_dir, flat_launch_dict, main_page_link, outhtml, outmarkdown ):
    for launch_id, launch_data in flat_launch_dict.items():
        launch_file = launch_data["file"]
        launch_file = os.path.normpath(launch_file)
        _LOGGER.info( "preparing page for item %s", launch_id )

        launch_list = []
        launch_includes = launch_data["included"]
        for include_data in launch_includes:
            include_id = get_launch_id(include_data)
            include_file = include_data["file"]
            include_file = os.path.normpath(include_file)
            item_link = prepare_item_link( include_id, include_file, True, "" )
            launch_list.append(item_link)
        launch_list.sort()

        params_data = launch_data["params"]

        fatten_list = []
        for param_data in params_data.values():
            par_key = param_data["key"]
            par_val = param_data["value"]
            fatten_list.append( (par_key, par_val) )
        fatten_list.sort( key=lambda item: item[0], reverse=False )
        params_list = prepare_params_list(fatten_list, sub_output_dir, "")

        nodes_list = launch_data["nodes"]
        for node_item in nodes_list:
            node_item["filename"] = os.path.abspath(node_item["filename"])
            remap_args = node_item["remap_args"]
            remap_args = convert_to_html(remap_args, shorten=False)
            node_item["remap_args"] = remap_args

        graph: Graph = generate_full_graph(flat_launch_dict)
        set_node_html_attribs( graph, "" )
        set_nodes_style( graph, [launch_id], style_dict=DEFAULT_ACTIVE_NODE_STYLE )
        item_out_path = prepare_filesystem_name( launch_id )
        graph.setName(item_out_path)

        resolve_dict = launch_data["resolve_dict"]
        args_dict = resolve_dict.get("arg", {})
        args_dict = dict(sorted(args_dict.items()))     # sort dict by keys

        item_dict = { "launch_file": launch_file,
                      "main_page_link": main_page_link,
                      "graph": graph,
                      "launch_list": launch_list,
                      "resolved_args": args_dict,
                      "nodes_list": nodes_list,
                      "params_list": params_list
                      }

        if outhtml:
            template_item = "roslaunchgraph/launchgraph_launch.html"
            generate_from_template( sub_output_dir, item_dict, template_name=template_item )

        if outmarkdown:
            template_item = "roslaunchgraph/launchgraph_launch.md"
            generate_from_template( sub_output_dir, item_dict, template_name=template_item )


def get_nodes_files_deps(launch_dict, out_subdir):
    ret_list = []

    launch_id = get_launch_id(launch_dict)
    nodes_list = launch_dict["nodes"]
    for node_data in nodes_list:
        node_name = node_data["name"]
        node_link_data = prepare_item_link( launch_id, node_name, True, out_subdir )
        ret_list.append( node_link_data )

    ret_list.sort()
    return ret_list


def get_flat_dict(launch_dict):
    launch_id = get_launch_id(launch_dict)
    ret_dict = { launch_id: launch_dict }

    included_list = launch_dict["included"]
    for included_item in included_list:
        sub_dict = get_flat_dict(included_item)
        ret_dict.update(sub_dict)

    return ret_dict


def get_launch_id(launch_item):
    launch_file = launch_item["file"]
    launch_file = os.path.normpath(launch_file)
    master_key = get_launch_args_string(launch_item)
    if not master_key:
        return launch_file
    # use hash instead of direct args to prevent "File name too long" exception
    encoded_args = master_key.encode('utf-8')
    master_key = hashlib.md5(encoded_args).hexdigest()
    return f"{launch_file}_{master_key}"


def get_launch_args_string(launch_item):
    launch_args = launch_item["launch_args"]
    if not launch_args:
        return ""
    params_list = []
    resolved_args = launch_item["resolve_dict"]["arg"]
    for arg in launch_args:
        val = resolved_args[arg]
        params_list.append(f"{arg}_{val}")
    ret_string = "_".join(params_list)
    ret_string = ret_string.replace(" ", "_")
    return ret_string


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
    flat_launch_dict = get_flat_dict(launch_dict)

    main_graph: Graph = generate_full_graph(flat_launch_dict)
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
