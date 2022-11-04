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

import logging
from typing import List

import pydotplus
from pydotplus.graphviz import quote_if_necessary


_LOGGER = logging.getLogger(__name__)


class Graph():
    
    def __init__( self, base_object=None ):
        if base_object is None:
            base_object = pydotplus.graphviz.Dot()
        self.base_graph = base_object

    def base(self) -> pydotplus.Graph:
        return self.base_graph

    def setAsSubgraph(self):
        self.base_graph = pydotplus.Subgraph()
        self.base_graph.set_name('')

    def getNodesCount(self):
        nodes = self.getNodesAll()
        return len( nodes )

    def getEdgesCount(self):
        edges = self.getEdgesAll()
        return len( edges )

    def getNodesAll(self) -> List[ pydotplus.Node ]:
        return get_nodes_all( self.base_graph )

    ## get nodes without destination edge
    def getNodesTop(self) -> List[ pydotplus.Node ]:
        edge_nodes = set()
        edges_list = self.getEdgesAll()
        for edge in edges_list:
            ## get destination nodes from edges
            port_name = edge.get_destination()
            edge_nodes.add( port_name )
        all_nodes = self.getNodesAll()
        ret_nodes = []
        for node in all_nodes:
            node_name = node.get_name()
            if node_name not in edge_nodes:
                ret_nodes.append( node )
        return ret_nodes

    ## get nodes without source edge
    def getNodesBottom(self) -> List[ pydotplus.Node ]:
        edge_nodes = set()
        edges_list = self.getEdgesAll()
        for edge in edges_list:
            ## get source nodes from edges
            port_name = edge.get_source()
            edge_nodes.add( port_name )
        all_nodes = self.getNodesAll()
        ret_nodes = []
        for node in all_nodes:
            node_name = node.get_name()
            if node_name not in edge_nodes:
                ret_nodes.append( node )
        return ret_nodes

    def getNode(self, node_name: str):
        qname = quote_if_necessary( node_name )
        nodes_list = self.base_graph.get_node( qname )
        if len(nodes_list) != 1:
            return None
        return nodes_list[0]

    def addNodes(self, nodes_list: List[pydotplus.Node] ):
        for node in nodes_list:
            node_name = node.get_name()
            self.addNode( node_name )

    def addNode( self, node_name: str, shape: str=None ) -> pydotplus.Node:
        found_node = self.getNode( node_name )
        if found_node is not None:
            ## node already added
            return None
        node = pydotplus.Node( node_name )
        if shape is not None:
            node.set( "shape", shape )
        self.base_graph.add_node( node )
        return node

    def removeNode(self, node_name, remove_edges=True):
        removed = remove_nodes_recursive( self.base_graph, node_name )
        if removed and remove_edges:
            self.removeEdgesFromNode( node_name )
        return removed

    def removeNodes(self, nodes_list: List[pydotplus.Node] ):
        removed = False
        for node in nodes_list:
            node_name = node.get_name()
            if self.removeNode( node_name ):
                removed = True
        return removed

    def getEdgesAll(self) -> List[ pydotplus.Edge ]:
        return self.base_graph.get_edges()

    def addEdge( self, from_node: str, to_node: str, create_nodes=False ):
        node_from = self.getNode( from_node )
        if node_from is None:
            if create_nodes == False:
                _LOGGER.warning( "unable to find from node >%s<", from_node )
                return None
            node_from = self.addNode( from_node )
            
        node_to   = self.getNode( to_node )
        if node_to is None:
            if create_nodes == False:
                _LOGGER.warning( "unable to find to node >%s<", to_node )
                return None
            node_to = self.addNode( to_node )
            
        new_edge = pydotplus.Edge( node_from, node_to )
        self.base_graph.add_edge( new_edge )

    def removeEdge( self, from_node: str, to_node: str ):
        return remove_edge_raw( self.base_graph, from_node, to_node )

    def removeEdgesFromNode( self, node_name: str ):
        remove_edges_recursive( self.base_graph, node_name )

    def setNodesRank( self, nodes_list: List[ pydotplus.Node ], rank: str ):
        sub_graph = Graph()
        sub_graph.setAsSubgraph()
        sub_base = sub_graph.base()
        for node in nodes_list:
            self.detachNodeRaw( node )
            sub_base.add_node( node )
        self.base_graph.add_subgraph( sub_base )
        sub_base.set_rank( rank )
        return sub_graph 
    
    ## remove node from graph, do not edit edges
    def detachNodeRaw( self, node: pydotplus.Node ):
        return detach_node_recursive( self.base_graph, node )

    ### =================================================================

    def toString(self):
        return self.base_graph.to_string()

    def writeRAW( self, file_path ):
        self.base_graph.write_raw( file_path ) 

    def writePNG( self, file_path ):
        self.base_graph.write_png( file_path ) 
        
    def write( self, file_path, file_format='png'):
        self.base_graph.write( file_path, format=file_format ) 


### =================================================================


def unquote_name( node_name ):
    return node_name.strip('\"')


def get_nodes_names( nodes_list: List[ pydotplus.Node ] ):
    return [ unquote_name( node.get_name() ) for node in nodes_list ] 


def get_nodes_all( graph ):
    nodes_list = graph.get_nodes()
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_nodes_all( sub )
        nodes_list.extend( sub_list )
    return nodes_list


def remove_nodes_recursive( graph, node_name ):
    qname = quote_if_necessary( node_name )
    removed = False
    if graph.del_node( qname ):
        removed = True
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        if remove_nodes_recursive( sub, node_name ):
            removed = True
    return removed


def detach_node_recursive( graph, node: pydotplus.Node ):
    ret_detached = False
    node_name  = node.get_name()
    nodes_list = graph.obj_dict['nodes'].get( node_name, None )
    if nodes_list is not None:
        node_seq = node.obj_dict['sequence']
        for item in nodes_list:
            item_seq = item['sequence']
            if item_seq == node_seq:
                nodes_list.remove( item )
                ret_detached = True
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        if detach_node_recursive( sub, node ):
            ret_detached = True
    return ret_detached


def remove_edges_recursive( graph, node_name ):
    qname = quote_if_necessary( node_name )
    edges_list: List[ pydotplus.Edge ] = graph.get_edges()
    for edge in edges_list:
        source_node_name = edge.get_source()
        dest_node_name   = edge.get_destination()
        if source_node_name == qname or dest_node_name == qname:
            remove_edge_raw( graph, source_node_name, dest_node_name )
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        remove_edges_recursive( sub, node_name )


def remove_edge_raw( graph, from_node: str, to_node: str ):
    qfrom = quote_if_necessary( from_node )
    qto   = quote_if_necessary( to_node )
    return graph.del_edge( qfrom, qto )
