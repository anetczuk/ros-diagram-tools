# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
from typing import List, Set
import hashlib

import pydotplus
from pydotplus.graphviz import quote_if_necessary, graph_from_dot_data

from rosdiagram.io import read_list
from rosdiagram.utils import get_create_item


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


EDGE_COLORS_PATH = os.path.join( SCRIPT_DIR, "graphviz_edge_colors.txt" )
EDGE_COLORS_LIST = read_list( EDGE_COLORS_PATH )


class Graph():

    def __init__( self, base_object=None ):
        if base_object is None:
            base_object = pydotplus.graphviz.Dot()
        self.base_graph = base_object

    def toJSON(self):
        return self.__str__()

    def base(self) -> pydotplus.Graph:
        return self.base_graph

    def parent(self) -> pydotplus.Graph:
        return self.base_graph.get_parent_graph()

    def empty(self):
        return self.getNodesCount() < 1

    def topGraph(self) -> pydotplus.Graph:
        curr_graph = self.base_graph
        while True:
            parent_graph = curr_graph.get_parent_graph()
            if parent_graph is None:
                return curr_graph
            if parent_graph is curr_graph:
                return curr_graph
            curr_graph = parent_graph

    def setAsSubgraph(self):
        self.base_graph = pydotplus.Subgraph()
        self.base_graph.set_name('')

    def setEngine(self, prog):
        self.base_graph.set_prog( prog )

    def set(self, prop, value ):
        self.base_graph.set( prop, value )

    def setProp(self, prop, value ):
        ## alias
        self.set( prop, value )

    def getName( self ):
        graph_name = self.base_graph.get_name()
        return unquote_name( graph_name )

    def setName( self, new_name ):
        graph_name = quote_if_necessary( new_name )
        return self.base_graph.set_name( graph_name )

    def setAsCluster(self, name):
        self.setName( f"cluster_{name}" )

    def getNodesCount(self):
        names = get_node_names_all( self.base_graph )
        return len( names )

    def getEdgesCount(self):
        edges = self.getEdgesAll()
        return len( edges )

    ## ===================================================

    def getNodesAll(self) -> List[ pydotplus.Node ]:
        return get_nodes_all( self.base_graph )

    def getNodeNamesAll(self, unquote=True) -> Set[ str ]:
        names_list = get_node_names_all( self.base_graph )
        if unquote:
            names_list = unquote_name_list( names_list )
        return names_list

    def getNodeNamesDict(self):
        all_nodes = self.getNodesAll()
        return nodes_to_dict( all_nodes )

    def getNodesByName(self, node_names_list) -> List[ pydotplus.Node ]:
        if len( node_names_list ) < 1:
            return []
        nodes_list = get_nodes_all( self.base_graph )
        return filter_nodes( nodes_list, node_names_list )

    def getNodesByLabels( self, node_labels_list: List[str]) -> List[ pydotplus.Node ]:
        if len( node_labels_list ) < 1:
            return []
        ret_list = []
        nodes_list = get_nodes_all( self.base_graph )
        for node in nodes_list:
            node_label = node.get("label")
            if node_label is None or node_label == "":
                node_label = unquote_name( node.get_name() )
            if node_label in node_labels_list:
                ret_list.append( node )
        return ret_list

    def getNodeNamesByLabels( self, node_labels_list: List[str] ) -> List[ pydotplus.Node ]:
        nodes_list = self.getNodesByLabels( node_labels_list )
        return get_nodes_names( nodes_list )

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

    def getSourceNames(self, node_name, levels=0) -> List[ List[str] ]:
        if self.hasNode( node_name ) is False:
            return []
        all_edges = get_edges_pairs( self.base_graph )
        #all_edges = get_edges_all( self.base_graph )
        ret_list = []
        search_list = [ node_name ]
        for _ in range(0, levels + 1):
            neighbour_list = []
            for search_name in search_list:
                qname = quote_if_necessary( search_name )
                for edge in all_edges:
                    port_node_name = edge[1]
#                     port_node_name = edge.get_destination()
                    if port_node_name != qname:
                        continue
                    ## dst is node_name
                    target_name = edge[0]
#                     target_name = edge.get_source()
                    is_node = self.hasNode( target_name )
                    if is_node is False:
                        continue
                    neighbour_list.append( target_name )
            ret_list.append( neighbour_list )
            search_list = neighbour_list
        return ret_list

    def getDestinationNames(self, node_name, levels=0) -> List[ List[str] ]:
        if self.hasNode( node_name ) is False:
            return []
        all_edges = get_edges_pairs( self.base_graph )
        #all_edges = get_edges_all( self.base_graph )
        ret_list = []
        search_list = [ node_name ]
        for _ in range(0, levels + 1):
            neighbour_list = []
            for search_name in search_list:
                qname = quote_if_necessary( search_name )
                for edge in all_edges:
                    port_node_name = edge[0]
                    #port_node_name = edge.get_source()
                    if port_node_name != qname:
                        continue
                    ## src is node_name
                    target_name = edge[1]
                    #target_name = edge.get_destination()
                    is_node = self.hasNode( target_name )
                    if is_node is False:
                        continue
                    neighbour_list.append( target_name )
            ret_list.append( neighbour_list )
            search_list = neighbour_list
        return ret_list

    def hasNode( self, node_name ):
        return has_node( self.base_graph, node_name )

    def getNode(self, node_name: str):
        return get_node( self.base_graph, node_name )

    def addNodes(self, nodes_list: List[pydotplus.Node] ):
        for node in nodes_list:
            self.addNodeObject( node )

    def addNode( self, node_name: str, shape: str = None, label: str = None ) -> pydotplus.Node:
        found_node = has_node( self.base_graph, node_name )
        if found_node is True:
            ## node already added
            return None
        node = pydotplus.Node( node_name )
        if shape is not None:
            node.set( "shape", shape )
        if label is not None:
            node.set( "label", label )
        self.base_graph.add_node( node )
        return node

    def addNodeObject(self, node: pydotplus.Node ):
        self.base_graph.add_node( node )
        return node

    def removeNode(self, node_name, remove_edges=True):
        qname = quote_if_necessary( node_name )
        removed = remove_nodes_recursive( self.base_graph, [qname] )
        if removed and remove_edges:
            self.removeEdgesFromNode( qname )
        return removed

    def removeNodesByName(self, node_names_list: Set[str], remove_edges=True ):
        qlist = [ quote_if_necessary( node_name ) for node_name in node_names_list ]
        removed = remove_nodes_recursive( self.base_graph, qlist )
        if removed and remove_edges:
            remove_edges_recursive( self.base_graph, qlist )
        return removed

    def removeNodes(self, nodes_list: List[pydotplus.Node] ):
        removed = False
        for node in nodes_list:
            node_name = node.get_name()
            if self.removeNode( node_name ):
                removed = True
        return removed

    ## remove node from graph, do not edit edges
    def detachNodeRaw( self, node: pydotplus.Node ):
        return detach_node_recursive( self.base_graph, node )

    def detachNodesByName( self, names_list: List[ str ] ):
        nodes_list = self.getNodesByName( names_list )
        for node in nodes_list:
            self.detachNodeRaw( node )
        return nodes_list

    def takeNodes(self, names_list: List[ str ]):
        top_graph     = self.topGraph()
        wrapped_top   = Graph( top_graph )
        detached_list = wrapped_top.detachNodesByName( names_list )
        self.addNodes( detached_list )
        return detached_list

    ## ===================================================

    def getEdgesAll(self) -> List[ pydotplus.Edge ]:
        ## shouldn't be: return get_edges_all( self.base_graph )
        return self.base_graph.get_edge_list()

    def getEdges( self, name_from, name_to ) -> List[ pydotplus.Edge ]:
        ret_list = []
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_source()
            if port_name != name_from:
                continue
            port_name = item.get_destination()
            if port_name != name_to:
                continue
            ret_list.append( item )
        return ret_list

    def getEdgesFrom( self, name ) -> List[ pydotplus.Edge ]:
        ret_list = []
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_source()
            if port_name == name:
                ret_list.append( item )
        return ret_list

    def getEdgesFromDict(self):
        ret_dict = {}
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_source()
            edges_list = get_create_item( ret_dict, port_name, [] )
            edges_list.append( item )
        return ret_dict

    def getEdgesTo( self, name ) -> List[ pydotplus.Edge ]:
        ret_list = []
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_destination()
            if port_name == name:
                ret_list.append( item )
        return ret_list

    def getEdgesToDict(self):
        ret_dict = {}
        edges = self.getEdgesAll()
        for item in edges:
            port_name = item.get_destination()
            edges_list = get_create_item( ret_dict, port_name, [] )
            edges_list.append( item )
        return ret_dict

    def getEdgesDict(self):
        edges = self.getEdgesAll()
        return create_edges_dict( edges )

    def addEdge( self, from_node: str, to_node: str, create_nodes=False ) -> pydotplus.Edge:
        if self.hasNode(from_node) is False:
            if create_nodes is False:
                _LOGGER.warning( "unable to find from node >%s<", from_node )
                return None
            self.addNode( from_node )

        if self.hasNode(to_node) is False:
            if create_nodes is False:
                _LOGGER.warning( "unable to find to node >%s<", to_node )
                return None
            self.addNode( to_node )

        new_edge = pydotplus.Edge( from_node, to_node )
        self.base_graph.add_edge( new_edge )

        if EDGE_COLORS_LIST:
            edge_string = f"{from_node}-{to_node}"
            item_hash   = hashlib.sha256( edge_string.encode('utf-8') ).hexdigest()
            color_index = int( item_hash, 16 ) % len( EDGE_COLORS_LIST )
            edge_color  = EDGE_COLORS_LIST[ color_index ]
            new_edge.set( "color", edge_color )

        return new_edge

    def removeEdgeByObject( self, edge: pydotplus.Edge ):
        detach_edge_recursive( self.base_graph, edge )

    def removeEdgesFromNode( self, node_name: str ):
        qname = quote_if_necessary( node_name )
        remove_edges_recursive( self.base_graph, [qname] )

    def removeEdgesFromNodes( self, node_name_list: List[str] ):
        qlist = [ quote_if_necessary( node_name ) for node_name in node_name_list ]
        remove_edges_recursive( self.base_graph, qlist )

    ## ===================================================

    def createSubGraph( self ) -> "Graph":
        sub_graph = Graph()
        sub_graph.setAsSubgraph()
        sub_base = sub_graph.base()
        self.base_graph.add_subgraph( sub_base )
        return sub_graph

    ## rank: same min max <integer>
    def setNodesRank( self, nodes_list: List[ pydotplus.Node ], rank: str ) -> "Graph":
        sub_graph = Graph()
        sub_graph.setAsSubgraph()
        sub_base = sub_graph.base()
        for node in nodes_list:
            self.detachNodeRaw( node )
            sub_base.add_node( node )
        self.base_graph.add_subgraph( sub_base )
        sub_base.set_rank( rank )
        return sub_graph

    def setNodesRankByName( self, names_list: List[ str ], rank: str ) -> "Graph":
        nodes_list = self.getNodesByName( names_list )
        return self.setNodesRank( nodes_list, rank )

    ### =================================================================

    def clone(self):
        content = self.toString()
        graph_copy = Graph()
        graph_copy.fromString( content )
        return graph_copy

    def toString(self):
        return self.base_graph.to_string()

    def fromString( self, content ):
        self.base_graph = graph_from_dot_data( content )

    def writeRAW( self, file_path ):
        self.write( file_path, "raw" )

    def writePNG( self, file_path ):
        self.write( file_path, "png" )

    def writeMap( self, file_path ):
        self.write( file_path, "cmapx" )
        #self.base_graph.write( file_path, prog=self.base_graph.prog, format="cmapx" )

    def write( self, file_path, file_format='png'):
        self.base_graph.write( file_path, prog=self.base_graph.prog, format=file_format )


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
def preserve_neighbour_nodes( graph: Graph, nodes_start_list: List[str], level=0 ):
    found_node_names = [ quote_if_necessary( node_name ) for node_name in nodes_start_list ]
    if len( found_node_names ) < 1:
        return
    all_node_names = graph.getNodeNamesAll( unquote=False )

    preserve_edges = set()
    preserve_nodes = []
    preserve_nodes.extend( found_node_names )

    all_edges = graph.getEdgesAll()
    from_edges_dict, to_edges_dict = create_edges_dict( all_edges )

    for node_name in found_node_names:
        target_names = graph.getSourceNames( node_name, level )
        target_names.insert( 0, [node_name] )
        n_size = len( target_names )
        for i in range( 0, n_size - 1 ):
            from_nodes = target_names[i + 1]
            to_names   = target_names[i]
            for from_name in from_nodes:
                from_edges = from_edges_dict.get( from_name, [] )
                #from_edges = graph.getEdgesFrom( from_name )
                for ed in from_edges:
                    port_name = ed.get_destination()
                    if port_name in to_names:
                        preserve_edges.add( ed )
        for item in target_names:
            preserve_nodes.extend( item )

        target_names = graph.getDestinationNames( node_name, level )
        target_names.insert( 0, [node_name] )
        n_size = len( target_names )
        for i in range( 0, n_size - 1 ):
            from_names = target_names[i]
            to_nodes   = target_names[i + 1]
            for to_name in to_nodes:
                to_edges = to_edges_dict.get( to_name, [] )
                for ed in to_edges:
                    port_name = ed.get_source()
                    if port_name in from_names:
                        preserve_edges.add( ed )
        for item in target_names:
            preserve_nodes.extend( item )

    preserve_set   = set( preserve_nodes )
    rem_names      = all_node_names.difference( preserve_set )
    graph.removeNodesByName( rem_names )

    edges_set = set( all_edges )
    rem_edges = edges_set.difference( preserve_edges )
    for rem_edge in rem_edges:
        graph.removeEdgeByObject( rem_edge )


def create_edges_dict( edges_list ):
    from_dict = {}
    to_dict   = {}
    for item in edges_list:
        port_name  = item.get_source()
        edges_list = get_create_item( from_dict, port_name, [] )
        edges_list.append( item )

        port_name  = item.get_destination()
        edges_list = get_create_item( to_dict, port_name, [] )
        edges_list.append( item )
    return ( from_dict, to_dict )


def set_nodes_style( graph: Graph, names_list, style_dict=None ):
    if len(names_list) < 1:
        return
    if style_dict is None:
        return
    if len(style_dict) < 1:
        return
    nodes_list: List[ pydotplus.Node ] = graph.getNodesAll()
    for node in nodes_list:
        node_name = node.get_name()
        raw_name  = unquote_name( node_name )
        if raw_name in names_list:
            for key, val in style_dict.items():
                node.set( key, val )


### =================================================================


def get_node_label( node_obj: pydotplus.Node ):
    node_label = node_obj.get( "label" )
    if node_label is not None:
        return node_label
    return unquote_name( node_obj.get_name() )


def set_node_labels( graph, labels_dict, override=True ):
    if labels_dict is None or len( labels_dict ) < 1:
        return
    all_nodes = graph.getNodesAll()
    for node_obj in all_nodes:
        if override is False:
            curr_label = node_obj.get( "label" )
            if curr_label is not None:
                continue
        node_name = node_obj.get_name()
        raw_name  = unquote_name( node_name )
        label = labels_dict.get( raw_name, None )
        if label is None:
            continue
        node_obj.set( "label", label )


def unquote_name( node_name ):
    return node_name.strip('\"')


def unquote_name_list( name_list ):
    return [ unquote_name( name ) for name in name_list ]


def get_nodes_names( nodes_list: List[ pydotplus.Node ], unquote=True ):
    if unquote:
        return [ unquote_name( node.get_name() ) for node in nodes_list ]
    return [ node.get_name() for node in nodes_list ]


def has_node( graph, node_name ) -> bool:
    qname = quote_if_necessary( node_name )
    if qname in graph.obj_dict['nodes']:
        return True

    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        if has_node( sub, node_name ):
            return True

    return False


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
    items_list = graph.get_node_list()
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_nodes_all( sub )
        items_list.extend( sub_list )
    return items_list


def get_node_names_all( graph ) -> Set[str]:
    items_list = set()
    nodes_dict = graph.obj_dict['nodes']
    for key in nodes_dict:
        nodes_list = nodes_dict[ key ]
        if len( nodes_list ) < 1:
            continue
        items_list.add( key )
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_node_names_all( sub )
        items_list.update( sub_list )
    return items_list


## get nodes with matching names in 'filter_names_list'
def filter_nodes( nodes_list: List[ pydotplus.Node ], filter_names_list: List[str] ):
    ret_list = []
    for node in nodes_list:
        raw_name = unquote_name( node.get_name() )
        if raw_name in filter_names_list:
            ret_list.append( node )
    return ret_list


def nodes_to_dict( nodes_list ):
    nodes_dict = {}
    for node in nodes_list:
        node_name = node.get_name()
        node_name = unquote_name( node_name )
        nodes_dict[ node_name ] = node
    return nodes_dict


def remove_nodes_recursive( graph, node_name_list ):
    removed = False
    for node_name in node_name_list:
        if graph.del_node( node_name ):
            removed = True
    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        if remove_nodes_recursive( sub, node_name_list ):
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


## getting edge pairs is cheaper than getting edges
def get_edges_pairs( graph ):
    graph_edges_dict = graph.obj_dict['edges']
    ret_list = []
    for key in graph_edges_dict:
        ret_list.append( (key[0], key[1]) )

    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        sub_list = get_edges_pairs( sub )
        ret_list.extend( sub_list )

    return ret_list


def remove_edges_recursive( graph, node_names_list ):
    ##edges_list: List[ pydotplus.Edge ] = graph.get_edge_list()
    graph_edges_dict = graph.obj_dict['edges']
    rem_keys = set()
    for src_dest_pair_key in graph_edges_dict:
#     for edge in edges_list:
        for node_name in node_names_list:
#             source_node_name = edge.get_source()
#             dest_node_name   = edge.get_destination()
#             if node_name in (source_node_name, dest_node_name):
#                 remove_edge_raw( graph, source_node_name, dest_node_name )
            if node_name in src_dest_pair_key:
                rem_keys.add( src_dest_pair_key )
                #remove_edge_raw( graph, src_dest_pair_key[0], src_dest_pair_key[1] )

    for key in rem_keys:
        del graph_edges_dict[ key ]

    ## go recursive through subgraphs
    sub_list = graph.get_subgraph_list()
    for sub in sub_list:
        remove_edges_recursive( sub, node_names_list )


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
