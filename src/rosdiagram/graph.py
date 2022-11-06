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
from pydotplus.graphviz import quote_if_necessary, graph_from_dot_data


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

    def setEngine(self, prog):
        self.base_graph.set_prog( prog )

    def getNodesCount(self):
        nodes = self.getNodesAll()
        return len( nodes )

    def getEdgesCount(self):
        edges = self.getEdgesAll()
        return len( edges )

    def getNodesAll(self) -> List[ pydotplus.Node ]:
        return get_nodes_all( self.base_graph )

    def getNodesByName(self, node_names_list) -> List[ pydotplus.Node ]:
        if len( node_names_list ) < 1:
            return []
        nodes_list = get_nodes_all( self.base_graph )
        return filter_nodes( nodes_list, node_names_list )

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

    def getSourceNodes(self, node_name, levels=0) -> List[ pydotplus.Node ]:
        node = self.getNode( node_name )
        if node is None:
            return []
        all_edges = get_edges_all( self.base_graph )
        ret_list = []
        search_list = [ node ]
        for _ in range(0, levels + 1):
            neighbour_list = []
            for search in search_list:
                curr_name = search.get_name()
                qname = quote_if_necessary( curr_name )
                for edge in all_edges:
                    port_node_name = edge.get_destination()
                    if port_node_name != qname:
                        continue
                    ## dst is node_name
                    target_name = edge.get_source()
                    target_node = self.getNode( target_name )
                    if target_node is None:
                        continue
                    neighbour_list.append( target_node )
            ret_list.append( neighbour_list )
            search_list = neighbour_list
        return ret_list

    def getDestinationNodes(self, node_name, levels=0) -> List[ pydotplus.Node ]:
        node = self.getNode( node_name )
        if node is None:
            return []
        all_edges = get_edges_all( self.base_graph )
        ret_list = []
        search_list = [ node ]
        for _ in range(0, levels + 1):
            neighbour_list = []
            for search in search_list:
                curr_name = search.get_name()
                qname = quote_if_necessary( curr_name )
                for edge in all_edges:
                    port_node_name = edge.get_source()
                    if port_node_name != qname:
                        continue
                    ## src is node_name
                    target_name = edge.get_destination()
                    target_node = self.getNode( target_name )
                    if target_node is None:
                        continue
                    neighbour_list.append( target_node )
            ret_list.append( neighbour_list )
            search_list = neighbour_list
        return ret_list

    def getNode(self, node_name: str):
        return get_node( self.base_graph, node_name )

    def addNodes(self, nodes_list: List[pydotplus.Node] ):
        for node in nodes_list:
            node_name = node.get_name()
            self.addNode( node_name )

    def addNode( self, node_name: str, shape: str = None ) -> pydotplus.Node:
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

    def getEdgesFrom( self, name ) -> List[ pydotplus.Edge ]:
        ret_list = []
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_source()
            if port_name == name:
                ret_list.append( item )
        return ret_list

    def getEdgesTo( self, name ) -> List[ pydotplus.Edge ]:
        ret_list = []
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_destination()
            if port_name == name:
                ret_list.append( item )
        return ret_list

    def addEdge( self, from_node: str, to_node: str, create_nodes=False ):
        node_from = self.getNode( from_node )
        if node_from is None:
            if create_nodes is False:
                _LOGGER.warning( "unable to find from node >%s<", from_node )
                return None
            node_from = self.addNode( from_node )

        node_to   = self.getNode( to_node )
        if node_to is None:
            if create_nodes is False:
                _LOGGER.warning( "unable to find to node >%s<", to_node )
                return None
            node_to = self.addNode( to_node )

        new_edge = pydotplus.Edge( node_from, node_to )
        self.base_graph.add_edge( new_edge )
        return new_edge

    def removeEdgeByObject( self, edge: pydotplus.Edge ):
        detach_edge_recursive( self.base_graph, edge )

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

    def fromString( self, content ):
        self.base_graph = graph_from_dot_data( content )

    def writeRAW( self, file_path ):
        self.base_graph.write_raw( file_path )

    def writePNG( self, file_path ):
        self.base_graph.write_png( file_path )

    def write( self, file_path, file_format='png'):
        self.base_graph.write( file_path, format=file_format )


### =================================================================


## preserve subgraph with top nodes from 'top_nodes_list'
def preserve_top_subgraph( graph: Graph, top_nodes_list ):
    if len(top_nodes_list) < 1:
        return
    while True:
        top_nodes = graph.getNodesTop()
        found_nodes = filter_nodes( top_nodes, top_nodes_list )
        nodes_to_remove = [node for node in top_nodes if node not in found_nodes]
        removed = graph.removeNodes( nodes_to_remove )
        if removed is False:
            break


## preserve nodes from 'nodes_list' and nodes in 'level' distance
def preserve_neighbour_nodes( graph: Graph, nodes_list, level=0 ):
    found_nodes = graph.getNodesByName( nodes_list )
    if len( found_nodes ) < 1:
        return

    preserve_edges = set()
    preserve_nodes = []
    preserve_nodes.extend( found_nodes )

    for node in found_nodes:
        node_name = node.get_name()
        target_nodes = graph.getSourceNodes( node_name, level )
        target_nodes.insert( 0, [node] )
        n_size = len( target_nodes )
        for i in range( 0, n_size - 1 ):
            from_nodes = target_nodes[i + 1]
            to_nodes   = target_nodes[i]
            to_names   = get_nodes_names( to_nodes )
            for curr_node in from_nodes:
                curr_name = curr_node.get_name()
                edges = graph.getEdgesFrom( curr_name )
                for ed in edges:
                    port_name = unquote_name( ed.get_destination() )
                    if port_name in to_names:
                        preserve_edges.add( ed )
        for item in target_nodes:
            preserve_nodes.extend( item )

        target_nodes = graph.getDestinationNodes( node_name, level )
        target_nodes.insert( 0, [node] )
        n_size = len( target_nodes )
        for i in range( 0, n_size - 1 ):
            from_nodes = target_nodes[i]
            to_nodes   = target_nodes[i + 1]
            from_names = get_nodes_names( from_nodes )
            for curr_node in to_nodes:
                curr_name = curr_node.get_name()
                edges = graph.getEdgesTo( curr_name )
                for ed in edges:
                    port_name = unquote_name( ed.get_source() )
                    if port_name in from_names:
                        preserve_edges.add( ed )
        for item in target_nodes:
            preserve_nodes.extend( item )

    all_nodes    = set( get_nodes_names( graph.getNodesAll() ) )
    preserve_set = set( get_nodes_names( preserve_nodes ) )
    rem_names    = all_nodes.difference( preserve_set )
    for rem_name in rem_names:
        graph.removeNode( rem_name )

    all_edges = set( graph.getEdgesAll() )
    rem_edges = all_edges.difference( preserve_edges )
    for rem_edge in rem_edges:
        graph.removeEdgeByObject( rem_edge )


### =================================================================


def unquote_name( node_name ):
    return node_name.strip('\"')


def get_nodes_names( nodes_list: List[ pydotplus.Node ] ):
    return [ unquote_name( node.get_name() ) for node in nodes_list ]


def get_nodes( graph, node_name ):
    ret_list = []
    qname = quote_if_necessary( node_name )
    nodes_list = graph.get_node( qname )
    ret_list.extend( nodes_list )
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_nodes( sub, node_name )
        ret_list.extend( sub_list )
    return ret_list


def get_node( graph, node_name ):
    nodes_list = get_nodes( graph, node_name )
    if len(nodes_list) != 1:
        return None
    return nodes_list[0]


def get_nodes_all( graph ):
    items_list = graph.get_nodes()
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_nodes_all( sub )
        items_list.extend( sub_list )
    return items_list


## get nodes with matching names in 'filter_names_list'
def filter_nodes( nodes_list: List[ pydotplus.Node ], filter_names_list: List[str] ):
    ret_list = []
    for node in nodes_list:
        raw_name = unquote_name( node.get_name() )
        if raw_name in filter_names_list:
            ret_list.append( node )
    return ret_list


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


def get_edges_all( graph ):
    items_list = graph.get_edges()
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_edges_all( sub )
        items_list.extend( sub_list )
    return items_list


def remove_edges_recursive( graph, node_name ):
    qname = quote_if_necessary( node_name )
    edges_list: List[ pydotplus.Edge ] = graph.get_edges()
    for edge in edges_list:
        source_node_name = edge.get_source()
        dest_node_name   = edge.get_destination()
        if qname in (source_node_name, dest_node_name):
            remove_edge_raw( graph, source_node_name, dest_node_name )
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        remove_edges_recursive( sub, node_name )


def detach_edge_recursive( graph, edge: pydotplus.Edge ):
    ret_detached = False
    edge_points = ( edge.get_source(), edge.get_destination() )
    edges_list = graph.obj_dict['edges'].get( edge_points, None )
    if edges_list is not None:
        node_seq = edge.obj_dict['sequence']
        for item in edges_list:
            item_seq = item['sequence']
            if item_seq == node_seq:
                edges_list.remove( item )
                ret_detached = True
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        if detach_edge_recursive( sub, edge ):
            ret_detached = True
    return ret_detached


def remove_edge_raw( graph, from_node: str, to_node: str ):
    qfrom = quote_if_necessary( from_node )
    qto   = quote_if_necessary( to_node )
    return graph.del_edge( qfrom, qto )
