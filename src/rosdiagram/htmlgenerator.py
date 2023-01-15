# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from rosdiagram.io import read_file, prepare_filesystem_name
from rosdiagram.graphviz import Graph, \
    unquote_name, set_nodes_style, unquote_name_list, \
    get_node_label
from rosdiagram import texttemplate


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


LABEL_DICT_KEY       = "label_dict"
NEIGHBOURS_RANGE_KEY = "neighbours_range"


def generate_graph_html( output_dir, params_dict=None ):
    if params_dict is None:
        params_dict = {}
    generator = HtmlGenerator( params_dict )
    generator.output_dir = output_dir
    generator.generate()


## =============================================================


class HtmlGenerator():

    OUTPUT_NODES_REL_DIR = os.path.join( "nodes" )

    def __init__(self, params_dict=None):
        if isinstance( params_dict, ParamsDict ) is False:
            params_dict   = ParamsDict( params=params_dict )
        self.params       = params_dict
        self.labels_dict  = self.params.get( "labels_dict", {} )
        self.output_dir   = None

        self._main_graph  = None

    def generate(self):
#         params_out_file = os.path.join( self.output_dir, "params.txt" )
#         params_dict = self.params.getDict()
#         write_dict( params_dict, params_out_file, 4 )

        self._main_graph = self._getMainGraph()
        set_node_html_attribs( self._main_graph, self.OUTPUT_NODES_REL_DIR )
        self._generateGraphMainPage()

        self._generateNodes()

    ## generate and store neighbour graphs
    def _generateNodes( self ):
        sub_items_dict = self.params.get( "sub_pages", {} )

        active_node_style = self.params.getValue( "active_node_style", DEFAULT_ACTIVE_NODE_STYLE )

        all_items = set( sub_items_dict.keys() )
        for node_id in all_items:
            _LOGGER.info( "preparing page for item %s", node_id )

            ## generate subgraph
            node_graph: Graph = self._getNodeGraph( node_id )

            ### set rank for neighbour nodes
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

            set_nodes_style( node_graph, [node_id], style_dict=active_node_style )
            set_node_html_attribs( node_graph, "", filter_nodes=all_items )

            self._generateGraphNodePage( node_graph, node_id )

    def _generateGraphMainPage( self ):
        main_item_dict = self.params.get( "main_page", {} )
        self._generateGraphPage( self._main_graph, main_item_dict, self.output_dir )

    def _generateGraphNodePage( self, graph, node_id ):
        node_out_dir = os.path.join( self.output_dir, self.OUTPUT_NODES_REL_DIR )
        os.makedirs( node_out_dir, exist_ok=True )

        sub_items_dict   = self.params.get( "sub_pages", {} )
        node_config_dict = sub_items_dict.get( node_id, {} )
        self._generateGraphPage( graph, node_config_dict, node_out_dir )

    def _generateGraphPage( self, graph, item_config_dict, output_dir ):
        page_params = item_config_dict.copy()

        ## ensure field
        page_params[ "item_type" ]      = page_params.get( "item_type", "" )
        page_params[ "head_css_style" ] = page_params.get( "head_css_style", "" )
        page_params[ "top_content" ]    = page_params.get( "top_content", "" )
        page_params[ "bottom_content" ] = page_params.get( "bottom_content", "" )

        is_mainpage = self.output_dir == output_dir

        store_graph_to_html( graph, output_dir )

        graph_id       = graph.getName()                       ## usually node id
        graph_filename = prepare_filesystem_name( graph_id )

        graph_label = self.labels_dict.get( graph_id, graph_id )

        graph_map = ""
        graph_image_path = ""
        if not graph.empty():
            map_out          = os.path.join( output_dir, graph_filename + ".map" )
            graph_map        = read_file( map_out )
            graph_image_path = f"{graph_filename}.png"

        alt_text = graph_label
#         if len(self.type_label) > 0:
#             alt_text = self.type_label + " " + alt_text

        link_subdir = ""
        if is_mainpage:
            link_subdir = self.OUTPUT_NODES_REL_DIR

        listener = ""
        listener_id = item_config_dict.get( "svr_listener", "" )
        if listener_id:
            link_list = self._getROSItemLinkList( [listener_id], link_subdir )
            if link_list:
                listener = link_list[0]

        converted_lists = []
        items_lists = item_config_dict.get( "lists", [] )
        for list_dict in items_lists:
            title = list_dict.get( "title", "Items" )
            items = list_dict.get( "items", [] )
            converted_list = self._getROSItemLinkList( items, link_subdir )
            converted_dict = { "title": title, "items": converted_list }
            converted_lists.append( converted_dict )

        main_page_link = ""
        if not is_mainpage:
            full_graph_name = self._main_graph.getName()
            item_filename   = prepare_filesystem_name( full_graph_name )
            main_page_link  = os.path.join( os.pardir, item_filename + ".html" )

        ## prepare input for template
        page_params.update( {   "body_color":       self._getStyle( "body_color", "#bbbbbb" ),
                                "main_page_link":   main_page_link,

                                "lists": converted_lists,

                                ## graph image specific fields
                                "graph_name":        graph_id,
                                "graph_image_path":  graph_image_path,
                                "alt_text":          alt_text,
                                "graph_map":         graph_map,

                                ## service specific fields
                                "srv_listener": listener
                                } )

        template_path = os.path.join( SCRIPT_DIR, "template", "nodegraph_page.html.tmpl" )
        html_out      = os.path.join( output_dir, graph_filename + ".html" )

        texttemplate.generate( template_path, html_out, INPUT_DICT=page_params )

    def _getMainGraph(self) -> Graph:
        main_dict = self.params.get( "main_page", {} )
        graph     = main_dict.get( "graph", None )
        if graph is None:
            return None
        graph.setName( "full_graph" )
        return graph

    def _getNodeGraph(self, node_id) -> Graph:
        sub_items_dict = self.params.get( "sub_pages", {} )
        item_dict      = sub_items_dict.get( node_id, {} )
        node_graph     = item_dict.get( "graph", None )
        if node_graph is None:
            raise RuntimeError( f"'graph' not set for item '{node_id}'" )
        node_graph.setName( node_id )
        return node_graph

    def _getROSItemLinkList(self, item_list, link_subdir="" ):
        ret_list = []
        for item_id in item_list:
            label = self.labels_dict.get( item_id, item_id )
            if self._isSubPage( item_id ):
                item_filename = prepare_filesystem_name( item_id )
                if link_subdir:
                    ret_list.append( (label, f"{link_subdir}/{item_filename}.html") )
                else:
                    ret_list.append( (label, f"{item_filename}.html") )
            else:
                ret_list.append( (label, None) )
        return ret_list

    def _getStyle( self, item_id, default_value ):
        style_dict = self.params.get( "style", {} )
        return style_dict.get( item_id, default_value )

    def _isSubPage(self, item_id):
        sub_items_dict = self.params.get( "sub_pages", {} )
        return item_id in sub_items_dict


## =============================================================


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


DEFAULT_ACTIVE_NODE_STYLE = { "style": "filled",
                              "fillcolor": "brown1"
                              }


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


## ============================================================================


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
