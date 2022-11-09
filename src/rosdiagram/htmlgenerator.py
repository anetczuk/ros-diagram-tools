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

from typing import List
import pydotplus

from rosdiagram.io import read_file, write_file, prepare_filesystem_name
from rosdiagram.graph import Graph, get_nodes_names, preserve_neighbour_nodes,\
    unquote_name


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def generate_graph_html( nodes_dict, graph_generator, output_dir ):
    generator = HtmlGenerator()
    generator.graph_factory   = lambda: graph_generator( nodes_dict )
    generator.output_root_dir = output_dir

    generator.generate()


##
class HtmlGenerator():

    def __init__(self):
        self.graph_factory         = None
        self.output_root_dir       = None
        self.output_nodes_rel_dir  = os.path.join( "nodes" )

        self.main_graph            = None
        self.output_nodes_dir      = None

    def generate( self ):
        self.generateMain()
        self.generateNodes()

    def generateMain( self, graph_name: str = None ):
        self._setMainGraph( graph_name )

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

        all_nodes = self.main_graph.getNodesAll()
        all_names = get_nodes_names( all_nodes )
        for item in all_names:
            item_filename = item.replace( "/", "_" )

            ## generate subgraph
            node_graph = self.graph_factory()
            node_graph.setName( item_filename )
            preserve_neighbour_nodes( node_graph, [item], 1 )
            paint_nodes( node_graph, [item] )
            set_node_html_attribs( node_graph, "" )

            self.prepareNodePage( node_graph, back_link )

    def _setMainGraph( self, graph_name: str = None ):
        if self.main_graph is None:
            self.main_graph = self.graph_factory()

        if graph_name is None:
            self.main_graph.setName( "full_graph" )
        else:
            self.main_graph.setName( graph_name )

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
        store_graph_html( self.graph, self.output_dir )

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


## ============================================================================


def store_graph_html( graph, output_dir ):
    graph_name = graph.getName()
    data_out = os.path.join( output_dir, graph_name + ".gv.txt" )
    graph.writeRAW( data_out )
    data_out = os.path.join( output_dir, graph_name + ".png" )
    graph.writePNG( data_out )
    data_out = os.path.join( output_dir, graph_name + ".map" )
    graph.writeMap( data_out )


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


def paint_nodes( graph: Graph, paint_list ):
    nodes_list: List[ pydotplus.Node ] = graph.getNodesAll()
    for node in nodes_list:
#         if node.get("shape") == "box":
#             node.set( "style", "filled" )
#             node.set( "fillcolor", "yellow" )

        node_name = node.get_name()
        raw_name  = unquote_name( node_name )

#         if raw_name.startswith( "/vb" ):
#             node.set( "style", "filled" )
#             node.set( "fillcolor", "yellow" )

#         if "_msgs" in raw_name:
#             node.set( "style", "filled" )
#             node.set( "fillcolor", "lightgreen" )
#         if "_srvs" in raw_name:
#             node.set( "style", "filled" )
#             node.set( "fillcolor", "lightblue" )
        if len(paint_list) > 0:
            if raw_name in paint_list:
                node.set( "style", "filled" )
                node.set( "fillcolor", "yellow" )
