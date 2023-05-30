# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse

from typing import Dict, Any

from rosdiagram.ros import rostopicdata
from rosdiagram.ros import rosservicedata

from showgraph.io import read_list
from showgraph.graphviz import Graph, set_node_labels, preserve_neighbour_nodes, set_nodes_style

from rosdiagram.utils import get_create_item
from rosdiagram.ros.rosnodedata import get_topics, get_services,\
    get_names_from_list, create_topics_dict, fix_names, split_to_groups,\
    get_services_info, filter_nodes, filter_topics,\
    get_services_from_dict, read_nodes, ROSNodeData, get_topics_info, filter_ros_nodes_dict,\
    get_topics_dict, get_services_dict
from rosdiagram.graphviztohtml import generate_graph_html
from rosdiagram.ros.rostopicdata import read_topics, filter_ros_topics_dict
from rosdiagram.ros.rosservicedata import read_services


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def painter_wrapper( graph: Graph, highlight_list, base_painter=None ):
    if base_painter:
        base_painter()
    if highlight_list:
        style = { "style": "filled",
                  "fillcolor": "yellow"
                  }
        set_nodes_style( graph, highlight_list, style )


## ===================================================================


def generate_graph( nodes_dict, labels_dict=None, show_services=True, full_graph=True, paint_function=None ):
    graph: Graph = None
    if full_graph:
        graph = generate_full_graph( nodes_dict, labels_dict=labels_dict, services_as_labels=show_services )
    else:
        graph = generate_compact_graph( nodes_dict, show_services=show_services, labels_dict=labels_dict )
    if paint_function:
        paint_function( graph )
    return graph


def generate_full_graph( nodes_dict, labels_dict=None, services_as_labels=True, services_as_nodes=False ) -> Graph:
    """ Generate graph with services represented as separate graph nodes. """

    if labels_dict is None:
        labels_dict = {}
    dot_graph = Graph()
    dot_graph.setName( "nodes_graph" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for node_id, lists in nodes_dict.items():
        node_obj   = dot_graph.addNode( node_id, shape="box" )
        node_label = labels_dict.get( node_id, node_id )
        node_obj.set( "label", node_label )

        topics: list = get_topics( lists )
        for topic_id in topics:
            topic_obj   = dot_graph.addNode( topic_id, shape="ellipse" )
            if topic_obj is None:
                ## already added
                continue
            topic_label = labels_dict.get( topic_id, topic_id )
            topic_obj.set( "label", topic_label )

        if services_as_nodes:
            services: list = get_services( lists )
            for service_id in services:
                service_obj   = dot_graph.addNode( service_id, shape="hexagon" )
                service_label = labels_dict.get( service_id, service_id )
                service_obj.set( "label", service_label )

        if services_as_labels:
            servs_list = lists[ "servs" ]
            node_label = node_label + "\n" + str( len(servs_list) ) + " services"
            node_obj.set( "label", node_label )

    ## add edges
    for node_id, lists in nodes_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]
        pubs_list  = get_names_from_list( pubs_list )
        subs_list  = get_names_from_list( subs_list )
        for pub in pubs_list:
            dot_graph.addEdge( node_id, pub )
        for sub in subs_list:
            dot_graph.addEdge( sub, node_id )

        if services_as_nodes:
            servs_list = lists[ "servs" ]
            servs_list = get_names_from_list( servs_list )
            for serv in servs_list:
                dot_graph.addEdge( serv, node_id )

    return dot_graph


def generate_compact_graph( nodes_dict, show_services=True, labels_dict=None ) -> Graph:
    """ Generate graph with services represented as number inside node. """

    if labels_dict is None:
        labels_dict = {}
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for node_id, lists in nodes_dict.items():
        node_label = labels_dict.get( node_id, node_id )
        node_obj   = dot_graph.addNode( node_id, shape="box", label=node_label )
        if show_services:
            servs_list = lists[ "servs" ]
            node_label = node_label + "\n" + str( len(servs_list) ) + " services"
            node_obj.set( "label", node_label )

            # for item in servs_list:
            #     dot_graph.addNode( item, shape="hexagon" )
            #     dot_graph.addEdge( node, item )

    topics_dict = create_topics_dict( nodes_dict )

    ## create connections dict
    connections_dict: Dict[ Any, Any ] = {}
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
            # if counter < 2:
            #     continue
            edge.set( "label", str(counter) )
    return dot_graph


## =====================================================


def generate_pages( nodes_dict, out_dir, nodes_labels=None,
                    topics_dump_dir=None, msgs_dump_dir=None, services_dump_dir=None, srvs_dump_dir=None,
                    paint_function=None, main_full_graph=False
                    ):
    if nodes_labels is None:
        nodes_labels = fix_names( nodes_dict )

    topics_dict = read_topics( topics_dump_dir )
    if topics_dict is None:
        topics_dict = get_topics_dict( nodes_dict, nodes_labels )

    filter_ros_topics_dict( topics_dict )

    topic_labels = rostopicdata.fix_names( topics_dict )

    services_dict = read_services( services_dump_dir )
    if services_dict is None:
        services_dict = get_services_dict( nodes_dict, nodes_labels )

    services_labels = rosservicedata.fix_names( services_dict )

    nodes_data: ROSNodeData = ROSNodeData( nodes_dict )
    nodes_data.nodes_label_dict = nodes_labels

    all_nodes, all_topics, all_services = split_to_groups( nodes_dict )

    main_graph: Graph = generate_graph( nodes_dict, labels_dict=nodes_labels, full_graph=main_full_graph, paint_function=paint_function )

    nodes_subpages_dict    = generate_subpages_dict( nodes_dict, all_nodes, nodes_labels, 1,
                                                     paint_function=paint_function )
    topics_subpages_dict   = generate_subpages_dict( nodes_dict, all_topics, nodes_labels, 0,
                                                     paint_function=paint_function )
    services_subpages_dict = generate_subpages_dict( nodes_dict, all_services, nodes_labels, 0,
                                                     paint_function=paint_function )

    for _, node_data in nodes_subpages_dict.items():
        node_data[ "item_type" ] = "node"

    #topics_info = nodes_data.getTopicsInfo()
    topics_info = get_topics_info( nodes_dict, topics_dict, msgs_dump_dir )
    for topic_id, topic_data in topics_info.items():
        pubs_list  = topic_data.get( "pubs", [] )
        subs_list  = topic_data.get( "subs", [] )

        pubs_names = []
        if pubs_list:
            pubs_names = [ get_label( topic_labels, item_id, "<unknown>" ) for item_id in pubs_list ]
        subs_names = []
        if subs_list:
            subs_names = [ get_label( topic_labels, item_id, "<unknown>" ) for item_id in subs_list ]

        sub_dict = topics_subpages_dict[ topic_id ]
        sub_dict[ "item_type" ]   = "topic"
        sub_dict[ "topic_name" ]  = get_label( topic_labels, topic_id, "<unknown>" )
        sub_dict[ "topic_pubs" ]  = pubs_names
        sub_dict[ "topic_subs" ]  = subs_names
        sub_dict[ "msg_type" ]    = topic_data.get( "type", "" )
        sub_dict[ "msg_content" ] = topic_data.get( "content", "" )

    services_info = get_services_info( nodes_dict, services_dict, srvs_dump_dir )
    for service_id, service_data in services_info.items():
        sub_dict = services_subpages_dict[ service_id ]
        sub_dict[ "item_type" ]    = "service"
        sub_dict[ "srv_name" ]     = get_label( services_labels, service_id, "<unknown>" )
        sub_dict[ "svr_listener" ] = service_data.get( "listener", "" )
        sub_dict[ "msg_type" ]     = service_data.get( "type", "" )
        sub_dict[ "msg_content" ]  = service_data.get( "content", "" )

    sub_items = {}
    sub_items.update( nodes_subpages_dict )
    sub_items.update( topics_subpages_dict )
    sub_items.update( services_subpages_dict )

    params_dict = { "style": {},
                    "labels_dict": nodes_labels,
                    "main_page": { "graph": main_graph,
                                   "lists": generate_items_lists( all_nodes, all_topics, all_services )
                                   },
                    "sub_pages": sub_items
                    }

    os.makedirs( out_dir, exist_ok=True )
    generate_graph_html( out_dir, params_dict )


def get_label( label_dict, item_id, default_name="<unknown>" ):
    item_name = label_dict.get( item_id, None )
    if item_name:
        return item_name
    return default_name


def generate_subpages_dict( nodes_dict, items_list, label_dict, neighbour_range, paint_function=None ):
    sub_items = {}
    for item_id in items_list:
        item_dict = {}
        sub_items[ item_id ] = item_dict
        item_graph: Graph = generate_full_graph( nodes_dict, labels_dict=label_dict )
        preserve_neighbour_nodes( item_graph, [item_id], neighbour_range )
        if paint_function:
            paint_function( item_graph )

        graph_names = list( item_graph.getNodeNamesAll() )
        nodes_list  = sorted( filter_nodes( nodes_dict, graph_names ) )
        item_dict[ "graph" ]    = item_graph
        item_dict[ "msg_name" ]    = ""
        item_dict[ "msg_type" ]    = ""
        item_dict[ "msg_content" ] = ""

        topics_list   = sorted( filter_topics( nodes_dict, graph_names ) )
        services_list = sorted( get_services_from_dict( nodes_dict, [ item_id ] ) )

        group_lists = []

        ## get lists of node pubs, subs and servs
#         node_data = nodes_dict.get( item_id, None )
#         if node_data is not None:
#             node_pubs = node_data.get( "pubs", [] )
#             node_pubs = [ item[0] for item in node_pubs ]
#             node_subs = node_data.get( "subs", [] )
#             node_subs = [ item[0] for item in node_subs ]
#             node_srvs = node_data.get( "servs", [] )
#             node_srvs = [ item[0] for item in node_srvs ]
#             group_lists.append( { "title": "Publications", "items": node_pubs } )
#             group_lists.append( { "title": "Subscriptions", "items": node_subs } )
#             group_lists.append( { "title": "Services", "items": node_srvs } )

        graph_items = generate_items_lists( nodes_list, topics_list, services_list )
        group_lists.extend( graph_items )

        item_dict[ "lists" ] = group_lists

    return sub_items


def generate_items_lists( nodes_list, topics_list, services_list ):
    ret_list = [ { "title": "ROS nodes",
                   "items": nodes_list
                   },
                 { "title": "ROS topics",
                   "items": topics_list
                   },
                 { "title": "ROS services",
                   "items": services_list
                   }
                 ]
    return ret_list


## ===================================================================


def configure_parser( parser ):
    parser.description = 'rosnode connection graph'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--nodesdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosnode' output" )
    parser.add_argument( '--topicsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rostopic' output" )
    parser.add_argument( '--msgsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosmsg' output" )
    parser.add_argument( '--servicesdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosservice' output" )
    parser.add_argument( '--srvsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rossrv' output" )
    parser.add_argument( '--highlightitems', action='store', required=False, default="", help="File with list of items to highlight" )
    parser.add_argument( '-mfg', '--mainfullgraph', action='store_true', help="Generate main full graph instead of compact one" )
    parser.add_argument( '-iri', '--includerosinternals', action='store_true', help="Include ROS internal items like /rosout and /record_*" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help="Output HTML" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args, paint_function=None ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    nodes_dict = read_nodes( args.nodesdumppath )
    if not nodes_dict:
        _LOGGER.warning( "no data found in %s", args.nodesdumppath )
        return

    if not args.includerosinternals:
        filter_ros_nodes_dict( nodes_dict )

    labels_dict = fix_names( nodes_dict )
    # info_dict  = get_node_info_dict( nodes_dict, labels_dict, args.msgsdumppath, args.srvsdumppath )

    highlight_list = read_list( args.highlightitems )
    painter = lambda graph: painter_wrapper( graph, highlight_list, paint_function )

    if len( args.outraw ) > 0 or len( args.outpng ) > 0:
        _LOGGER.info( "generating main graph" )
        graph: Graph = generate_graph( nodes_dict, labels_dict=labels_dict, full_graph=args.mainfullgraph, paint_function=painter )
        if len( args.outraw ) > 0:
            graph.writeRAW( args.outraw )
        if len( args.outpng ) > 0:
            graph.writePNG( args.outpng )

    ##
    ## generate HTML data
    ##
    if args.outhtml and len( args.outdir ) > 0:
        _LOGGER.info( "generating HTML graph" )
        os.makedirs( args.outdir, exist_ok=True )
        generate_pages( nodes_dict, args.outdir,
                        nodes_labels=labels_dict,
                        topics_dump_dir=args.topicsdumppath,
                        msgs_dump_dir=args.msgsdumppath,
                        services_dump_dir=args.servicesdumppath,
                        srvs_dump_dir=args.srvsdumppath,
                        paint_function=painter,
                        main_full_graph=args.mainfullgraph
                        )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
