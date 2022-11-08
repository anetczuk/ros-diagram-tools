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
import sys
import logging

import re
from typing import Set

from rosdiagram.io import read_list, read_file
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
        self.graph_factory    = None
        self.output_root_dir  = None
        self.output_nodes_dir = None

    def generate( self ):
        self.output_nodes_dir = os.path.join( self.output_root_dir, "nodes" )
        os.makedirs( self.output_nodes_dir, exist_ok=True )
        
        full_graph = self.graph_factory()
        full_graph.setName( "full_graph" )
        self.set_node_html_attribs( full_graph, "nodes/" )
    
        self.store_graph_html( full_graph, self.output_root_dir )
        full_graph_name = full_graph.getName()
        self.prepare_graph_page( self.output_root_dir, full_graph_name )
        
        index_html = f"""<body>
    <a href="{full_graph_name}.html">big graph</a>
    </body>
    """
        index_out = os.path.join( self.output_root_dir, "index.html" )
        with open( index_out, 'w', encoding='utf-8' ) as content_file:
            content_file.write( index_html )
            
        self.generate_nodes( full_graph )
        
    ## generate and store neighbour graphs
    def generate_nodes( self, full_graph ):
        full_graph_name = full_graph.getName()
        all_nodes = full_graph.getNodesAll()
        all_names = get_nodes_names( all_nodes )

        for item in all_names:
            item_filename = item.replace( "/", "_" )
            
            node_graph = self.graph_factory()
            node_graph.setName( item_filename )
            preserve_neighbour_nodes( node_graph, [item], 0 )
            paint_nodes( node_graph, [item] )
            self.set_node_html_attribs( node_graph, "" )
            
            self.store_graph_html( node_graph, self.output_nodes_dir )
            self.prepare_node_page( item_filename, "../" + full_graph_name + ".html" )
    
    def set_node_html_attribs( self, graph, local_dir ):
        all_nodes = graph.getNodesAll()
        for node_obj in all_nodes:
            node_name = node_obj.get_name()
            raw_name  = unquote_name( node_name )
            node_obj.set( "tooltip", "node: " + raw_name )
            node_url = local_dir + raw_name.replace( "/", "_" ) + ".html"
            node_obj.set( "href", node_url )
    
    def store_graph_html( self, graph, output_dir ):
        graph_name = graph.getName()
        data_out = os.path.join( output_dir, graph_name + ".gv.txt" )
        graph.writeRAW( data_out )
        data_out = os.path.join( output_dir, graph_name + ".png" )
        graph.writePNG( data_out )
        data_out = os.path.join( output_dir, graph_name + ".map" )
        graph.writeMap( data_out )
        
    def prepare_graph_page( self, output_dir, graph_name ):
        map_out = os.path.join( output_dir, graph_name + ".map" )
        graph_map = read_file( map_out )
        if graph_map is None:
            _LOGGER.error( "unable to generate html page" )
            return

        index_html = f"""<body>
<img src="{graph_name}.png" alt="graph {graph_name}" usemap="#{graph_name}">
{graph_map}
</body>
"""
        html_out = os.path.join( output_dir, graph_name + ".html" )
        with open( html_out, 'w', encoding='utf-8' ) as content_file:
            content_file.write( index_html )
        
    def prepare_node_page( self, node_name, main_page_path ):
        map_out = os.path.join( self.output_nodes_dir, node_name + ".map" )
        graph_map = read_file( map_out )
        if graph_map is None:
            _LOGGER.error( "unable to generate html page" )
            return

        index_html = f"""<body>
<a href="{main_page_path}">back to big graph</a>
</br>
<img src="{node_name}.png" alt="node {node_name}" usemap="#{node_name}">
{graph_map}
</body>
"""
        html_out = os.path.join( self.output_nodes_dir, node_name + ".html" )
        with open( html_out, 'w', encoding='utf-8' ) as content_file:
            content_file.write( index_html )


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
