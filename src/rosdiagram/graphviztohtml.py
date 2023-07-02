# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from showgraph.io import read_file, prepare_filesystem_name
from showgraph.graphviz import Graph, \
    unquote_name, unquote_name_list, get_node_label

from rosdiagram import texttemplate


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


DEFAULT_ACTIVE_NODE_STYLE = { "style": "filled",
                              "fillcolor": "brown1"
                              }


## generate graph page
def generate_from_template( output_dir, params_dict=None, template_name="dotgraph_page.html" ):
    template_id = f"{template_name}.tmpl"
    generate( output_dir, params_dict, template_id )


## uses keys: "graph", "graph_label", "style", "labels_dict" (deprecated)
def generate( output_dir, params_dict, template_id="" ):
    page_params = params_dict.copy()

    node_graph = page_params.get( "graph" )
    if node_graph is None:
        raise RuntimeError( "'graph' not set in params dict" )

    store_graph_to_html( node_graph, output_dir )

    graph_id       = node_graph.getName()                       ## usually node id
    graph_filename = prepare_filesystem_name( graph_id )

    graph_map = ""
    graph_image_path = ""
    if not node_graph.empty():
        map_out   = os.path.join( output_dir, graph_filename + ".map" )
        graph_map = read_file( map_out )
        graph_image_path = f"{graph_filename}.png"

    graph_label = page_params.get( "graph_label", None )
    if graph_label is None:
        labels_dict  = page_params.get( "labels_dict", {} )
        graph_label = labels_dict.get( graph_id, graph_id )

    style_dict = page_params.get( "style", {} )

    ## prepare input for template
    page_params.update( {   "body_color":       style_dict.get( "body_color", "#bbbbbb" ),

                            ## graph image specific fields
                            "graph_name":           graph_id,
                            "graph_image_path":     graph_image_path,
                            "graph_image_alt_text": graph_label,
                            "graph_map":            graph_map,
                            } )
    page_params.setdefault( "head_css_style", "" )

    template_path    = os.path.join( SCRIPT_DIR, "template", template_id )
    output_html_file = os.path.join( output_dir, graph_filename + ".html" )

    texttemplate.generate( template_path, output_html_file, INPUT_DICT=page_params )


def store_graph_to_html( graph: Graph, output_dir ):
    if graph.empty():
        ## empty graph -- do not store
        return
    graph_name    = graph.getName()
    item_filename = prepare_filesystem_name( graph_name )

    # data_out = os.path.join( output_dir, item_filename + ".gv.txt" )
    # graph.writeRAW( data_out )
    data_out = os.path.join( output_dir, item_filename + ".png" )
    graph.writePNG( data_out )
    data_out = os.path.join( output_dir, item_filename + ".map" )
    graph.writeMap( data_out )


##
class ParamsDict():

    def __init__( self, params: dict = None ):
        self.params = params
        if self.params is None:
            self.params = {}

    def getDict(self):
        return self.params

    def get(self, key, default=None ):
        return self.params.get( key, default )

    def getValue(self, key, default=None ):
        key_value = self.params.get( key, None )
        if key_value is None:
            return default
        try:
            return key_value()
        except TypeError:
            pass
        return key_value

    def getNamed(self, key, name, default=None ):
        key_value = self.params.get( key, None )
        if key_value is None:
            return default
        try:
            return key_value( name )
        except TypeError:
            pass
        try:
            return key_value[ name ]
        except TypeError:
            pass
        return key_value


## ===================================================================


def convert_links_dict( items_lists, sub_items_list, labels_dict, link_subdir ):
    converted_lists = []
    for list_dict in items_lists:
        title = list_dict.get( "title", "Items" )
        items = list_dict.get( "items", [] )
        converted_list = convert_links_list( items, sub_items_list, labels_dict, link_subdir )
        converted_dict = { "title": title, "items": converted_list }
        converted_lists.append( converted_dict )
    return converted_lists


def convert_links_list( items_lists, sub_items_list, labels_dict, link_subdir ):
    converted_list = []
    for item_id in items_lists:
        label = labels_dict.get( item_id, item_id )
        is_subpage = item_id in sub_items_list
        item_link = prepare_item_link( item_id, label, is_subpage, link_subdir )
        converted_list.append( item_link )
    return converted_list


## returns pair: ( <label>, <URL> )
def prepare_item_link( item_id, label, is_subpage, link_subdir="" ):
    if not is_subpage:
        return (label, None)
    item_filename = prepare_filesystem_name( item_id )
    if link_subdir:
        return (label, f"{link_subdir}/{item_filename}.html")
    return (label, f"{item_filename}.html")


def set_node_graph_ranks( node_graph, node_id ):
    node_graph.setName( node_id )
    node_graph.setNodesRankByName( [node_id], 100 )

    source_layers = node_graph.getSourceNames( node_id, 0 )
    if len(source_layers) > 0:
        source_names = source_layers[0]
        source_names = unquote_name_list( source_names )
        node_graph.setNodesRankByName( source_names, 50 )

    destination_layers = node_graph.getDestinationNames( node_id, 0 )
    if len(destination_layers) > 0:
        destination_names = destination_layers[0]
        destination_names = unquote_name_list( destination_names )
        node_graph.setNodesRankByName( destination_names, 150 )


def set_node_html_attribs( graph, node_local_dir, filter_nodes=None ):
    local_dir = node_local_dir
    if len(local_dir) > 0:
        local_dir = local_dir + os.sep

    all_nodes = graph.getNodesAll()
    for node_obj in all_nodes:
        node_name = node_obj.get_name()
        raw_name  = unquote_name( node_name )
        if filter_nodes is not None:
            if raw_name not in filter_nodes:
                continue
        node_label = get_node_label( node_obj )
        node_obj.set( "tooltip", "node: " + node_label )
        node_filename = prepare_filesystem_name( raw_name )
        node_url = local_dir + node_filename + ".html"
        node_obj.set( "href", node_url )
