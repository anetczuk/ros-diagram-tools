# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse

from typing import List

from showgraph.graphviz import Graph
from rosdiagram.utils import get_create_item
from rosdiagram.ros.rostopicdata import fix_names, get_nodes, join_common_topics,\
    get_nodes_all, read_topics


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def generate( topic_info_dir ):
    data_dict = read_topics( topic_info_dir )
    if data_dict is None:
        data_dict = {}
    graph     = generate_graph( data_dict )
    return graph


def generate_graph( topics_dict ) -> Graph:
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    fix_names( topics_dict )

    ## add nodes
    for topic, lists in topics_dict.items():
        dot_graph.addNode( topic, shape="ellipse" )
        nodes: list = get_nodes( lists )
        for item in nodes:
            dot_graph.addNode( item, shape="box" )

    ## add edges
    for topic, lists in topics_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        for pub in pubs_list:
            dot_graph.addEdge( pub, topic )
        for sub in subs_list:
            dot_graph.addEdge( topic, sub )

    return dot_graph


def generate_nodes_graph( topics_dict ):
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    fix_names( topics_dict )

    ## add nodes
    for _, lists in topics_dict.items():
        nodes: list = get_nodes( lists )
        for item in nodes:
            dot_graph.addNode( item, shape="box" )

    ## create connections dict
    connections_dict = {}
    for _, lists in topics_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        for pub in pubs_list:
            pub_dict = get_create_item( connections_dict, pub, {} )
            for sub in subs_list:
                sub_counter = get_create_item( pub_dict, sub, 0 )
                #edge = dot_graph.addEdge( pub, sub )
                pub_dict[ sub ] = sub_counter + 1

    ## add edges
    for pub, subs_dict in connections_dict.items():
        for sub, counter in subs_dict.items():
            #edge = dot_graph.addEdge( pub, sub )
            edge = dot_graph.addEdge( pub, sub )
            if counter > 1:
                edge.set( "label", str(counter) )

    return dot_graph


def generate_common_graph( left_topics_dict, right_topics_dict, left_label: str = "", right_label: str = "" ) -> Graph:
    join_dict = join_common_topics( left_topics_dict, right_topics_dict )

    common_topics_set: List[str] = []
    for item in join_dict:
        common_topics_set.append( item )
    common_topics_set = list( dict.fromkeys(common_topics_set) )

    fix_names( left_topics_dict )
    fix_names( right_topics_dict )
    fix_names( join_dict )

    join_graph: Graph = generate_graph( join_dict )
    join_graph.set( "newrank", "true" )
    join_graph.setNodesRankByName( common_topics_set, "same" )

    left_nodes  = get_nodes_all( left_topics_dict )
    right_nodes = get_nodes_all( right_topics_dict )

    left_subgraph: Graph = join_graph.setNodesRankByName( left_nodes, "min" )
    left_subgraph.setAsCluster( "left")
    if left_label:
        left_subgraph.set( "label", left_label )

    right_subgraph: Graph = join_graph.setNodesRankByName( right_nodes, "max" )
    right_subgraph.setAsCluster( "right")
    if right_label:
        right_subgraph.set( "label", right_label )

    return join_graph


## ===================================================================


def configure_parser( parser ):
    parser.description = 'rostopic flow graph (tool is obsolete, use rosnodegraph)'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--topicsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rostopic list' output" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.WARNING )

    graph = generate( args.topicsdumppath )

    if len( args.outraw ) > 0:
        graph.writeRAW( args.outraw )
    if len( args.outpng ) > 0:
        graph.writePNG( args.outpng )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
