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
import logging

from rosdiagram.io import read_file, write_file, prepare_filesystem_name
from rosdiagram.graph import Graph, get_nodes_names, preserve_neighbour_nodes,\
    unquote_name, set_nodes_style


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def generate_graph_html( output_dir, params_dict=None ):
    if params_dict is None:
        params_dict = {}
    generator = HtmlGenerator( params_dict )
    generator.output_root_dir = output_dir
    generator.generate()


##
class HtmlGenerator():

    def __init__( self, params_dict=None ):
        self.params = params_dict
        if self.params is None:
            self.params = {}

        self.output_root_dir       = None
        self.output_nodes_rel_dir  = os.path.join( "nodes" )

        self.main_graph: Graph     = None
        self.output_nodes_dir      = None

    def generate( self ):
        self.generateMain()
        self.generateNodes()

    def generateMain( self, graph_name: str = None ):
        self._setMainGraph( graph_name )

        main_engine = self._getParamValue( "main_engine" )
        if main_engine is not None:
            self.main_graph.setEngine( main_engine )

        set_node_html_attribs( self.main_graph, self.output_nodes_rel_dir )
        self.prepareMainPage()

    ## generate and store neighbour graphs
    def generateNodes( self ):
        self._setMainGraph()

        self.output_nodes_dir = os.path.join( self.output_root_dir, self.output_nodes_rel_dir )
        os.makedirs( self.output_nodes_dir, exist_ok=True )

        full_graph_name = self.main_graph.getName()
        main_page_path  = os.path.join( os.pardir, full_graph_name + ".html" )
        back_link = f"""\
<a href="{main_page_path}">back to big graph</a>
<br />"""

        neighbours_range  = self.params.get( "neighbours_range", 0 )
        active_node_style = self.params.get( "active_node_style", DEFAULT_ACTIVE_NODE_STYLE )

        all_nodes = self.main_graph.getNodesAll()
        all_names = get_nodes_names( all_nodes )
        
        engine_map = self._getNodeEngineMap( all_names )
        
        for item in all_names:
            _LOGGER.info( "preparing page for node %s", item )
            item_filename = prepare_filesystem_name( item )

            ## generate subgraph
            node_graph: Graph = self._spawnGraph()
            
            engine = engine_map.get( item, None )
            if engine is not None:
                node_graph.setEngine( engine )
            
            node_graph.setName( item_filename )
            preserve_neighbour_nodes( node_graph, [item], neighbours_range )
            set_nodes_style( node_graph, [item], style_dict=active_node_style )
            set_node_html_attribs( node_graph, "" )

            self.prepareNodePage( node_graph, back_link )

    def _setMainGraph( self, graph_name: str = None ):
        if self.main_graph is None:
            self.main_graph = self._spawnGraph()

        if graph_name is None:
            self.main_graph.setName( "full_graph" )
        else:
            self.main_graph.setName( graph_name )

    def _spawnGraph( self ) -> Graph:
        graph_factory = self.params.get( "graph_factory", None )
        if graph_factory is None:
            raise RuntimeError( "graph_factory not set" )
        return graph_factory()

    def _getNodeEngineMap(self, nodes_list):
        engine_map = {}
        for node in nodes_list:
            node_engine = self._getParamNamed( "node_engine", node )
            if node_engine is None:
                node_engine = self._getParamValue( "main_engine" )
            if node_engine is not None:
                engine_map[ node ] = node_engine
        return engine_map

    def _getParamValue(self, key ):
        key_param = self.params.get( key, None )
        if key_param is None:
            return None
        try:
            return key_param()
        except:
            pass
        return key_param

    def _getParamNamed(self, key, name ):
        key_param = self.params.get( key, None )
        if key_param is None:
            return None
        try:
            return key_param( name )
        except:
            pass
        try:
            return key_param[ name ]
        except:
            pass
        return key_param

    ## ==================================================================

    def prepareMainPage( self ):
        generator = GraphHtmlGenerator( self.main_graph, self.output_root_dir )
        generator.type_label = "graph"

        generator.generate()

        ## index page
        graph_name = self.main_graph.getName()
        index_out  = os.path.join( self.output_root_dir, "index.html" )
        index_html = f"""\
<body>
    <a href="{graph_name}.html">big graph</a>
</body>
"""
        write_file( index_out, index_html )

    def prepareNodePage( self, node_graph, back_link="" ):
        generator = GraphHtmlGenerator( node_graph, self.output_nodes_dir )
        generator.graph_top_content = back_link
        generator.type_label = "node"

        generator.generate()


##
class GraphHtmlGenerator():

    def __init__(self, graph=None, output_dir=None ):
        self.graph_top_content     = ""
        self.graph_bottom_content  = ""
        self.type_label            = ""

        self.graph                 = graph
        self.output_dir            = output_dir

    def generate( self ):
        self.storeGraphHtml()

        graph_name = self.graph.getName()
        map_out    = os.path.join( self.output_dir, graph_name + ".map" )
        graph_map  = read_file( map_out )
        if graph_map is None:
            _LOGGER.error( "unable to generate html page" )
            return

        alt_text = graph_name
        if len(self.type_label) > 0:
            alt_text = self.type_label + " " + alt_text

        html_out   = os.path.join( self.output_dir, graph_name + ".html" )
        index_html = f"""\
<body>
{self.graph_top_content}
<img src="{graph_name}.png" alt="{alt_text}" usemap="#{graph_name}">
{graph_map}
{self.graph_bottom_content}
</body>
"""
        write_file( html_out, index_html )

    def storeGraphHtml( self ):
        graph_name = self.graph.getName()
    #     data_out = os.path.join( self.output_dir, graph_name + ".gv.txt" )
    #     self.graph.writeRAW( data_out )
        data_out = os.path.join( self.output_dir, graph_name + ".png" )
        self.graph.writePNG( data_out )
        data_out = os.path.join( self.output_dir, graph_name + ".map" )
        self.graph.writeMap( data_out )


DEFAULT_ACTIVE_NODE_STYLE = { "style": "filled",
                              "fillcolor": "brown1"
                              }


## ============================================================================


def set_node_html_attribs( graph, node_local_dir ):
    local_dir = node_local_dir
    if len(local_dir) > 0:
        local_dir = local_dir + os.sep

    all_nodes = graph.getNodesAll()
    for node_obj in all_nodes:
        node_name = node_obj.get_name()
        raw_name  = unquote_name( node_name )
        node_obj.set( "tooltip", "node: " + raw_name )
        node_filename = prepare_filesystem_name( raw_name )
        node_url = local_dir + node_filename + ".html"
        node_obj.set( "href", node_url )
