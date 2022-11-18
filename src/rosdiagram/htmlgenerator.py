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
        self.params = ParamsDict( params=params_dict )

        self.output_root_dir       = None
        self.output_nodes_rel_dir  = os.path.join( "nodes" )

        self.main_graph: Graph     = None
        self.output_nodes_dir      = None

    def generate( self ):
        self.generateMain()
        self.generateNodes()

    def generateMain( self, graph_name: str = None ):
        self._setMainGraph( graph_name )

        main_engine = self.params.get( "main_engine" )
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
<a href="{main_page_path}">back to Main graph</a>
<br />"""

        neighbours_range  = self.params.get( "neighbours_range", 0 )
        active_node_style = self.params.getValue( "active_node_style", DEFAULT_ACTIVE_NODE_STYLE )

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
            set_node_html_attribs( node_graph, "", filter_nodes=all_names )

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
            node_engine = self.params.getNamed( "node_engine", node )
            if node_engine is None:
                node_engine = self.params.getValue( "main_engine" )
            if node_engine is not None:
                engine_map[ node ] = node_engine
        return engine_map

    ## ==================================================================

    def prepareMainPage( self ):
        params_dict = self.params.getDict()
        generator = GraphHtmlGenerator( self.main_graph, self.output_root_dir, params_dict=params_dict )
        generator.graph_top_content = "Main graph"
        generator.type_label = "graph"

        generator.generate()

#         ## index page
#         graph_name = self.main_graph.getName()
#         index_out  = os.path.join( self.output_root_dir, "index.html" )
#         index_html = f"""\
# <body>
#     <a href="{graph_name}.html">big graph</a>
# </body>
# """
#         write_file( index_out, index_html )

    def prepareNodePage( self, node_graph, back_link="" ):
        params_dict = self.params.getDict()
        generator = GraphHtmlGenerator( node_graph, self.output_nodes_dir, params_dict=params_dict )
        generator.graph_top_content = back_link
        generator.type_label = "node"

        generator.generate()


##
class GraphHtmlGenerator():

    def __init__(self, graph=None, output_dir=None, params_dict=None ):
        self.params                = ParamsDict( params=params_dict )
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

        body_color     = self.params.get( "body_color", "#bbbbbb" )
        head_css_style = self.params.get( "head_css_style", "" )

        alt_text = graph_name
        if len(self.type_label) > 0:
            alt_text = self.type_label + " " + alt_text

        page_params = { "body_color":       body_color,
                        "head_css_style":   head_css_style,
                        "top_content":      self.graph_top_content,
                        "bottom_content":   self.graph_bottom_content,

                        "graph_name":   graph_name,
                        "alt_text":     alt_text,
                        "graph_map":    graph_map
                        }

        html_out   = os.path.join( self.output_dir, graph_name + ".html" )
        index_html = GRAPH_PAGE_TEMPLATE.format( **page_params )
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


GRAPH_PAGE_TEMPLATE = """\
<html>
<head>
<style>
    body {{ padding: 24;
            background-color: {body_color};
         }}
    .center_content {{ width: 100%;
                       margin-right: auto; margin-left: auto;
                       text-align: center;
                       padding-top: 24; padding-bottom: 24;
                    }}
{head_css_style}
</style>
</head>

<body>
    <div class="top_content">
{top_content}
    </div>
    <div class="center_content">
        <img src="{graph_name}.png" alt="{alt_text}" usemap="#{graph_name}">
{graph_map}
    </div>
    <div class="bottom_content">
{bottom_content}
    </div>
</body>

</html>
"""


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
        node_obj.set( "tooltip", "node: " + raw_name )
        node_filename = prepare_filesystem_name( raw_name )
        node_url = local_dir + node_filename + ".html"
        node_obj.set( "href", node_url )
