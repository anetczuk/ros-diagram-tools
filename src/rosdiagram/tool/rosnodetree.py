# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse

from rosdiagram.utils import get_create_item
from rosdiagram.ros.rosnodedata import get_topics, get_services,\
    get_names_from_list, create_topics_dict, fix_names, split_to_groups,\
    get_topics_info, get_services_info, filter_nodes, filter_topics,\
    get_services_from_dict, read_nodes
from rosdiagram.htmlgenerator import generate_graph_html
from rosdiagram.graphviz import Graph, set_node_labels, preserve_neighbour_nodes
from rosdiagram.ros.rosutils import remove_ros_items


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


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
            # if counter < 2:
            #     continue
            edge.set( "label", str(counter) )
    return dot_graph


## =====================================================


def generate_pages( nodes_dict, out_dir, config_params_dict=None ):
    if config_params_dict is None:
        config_params_dict = {}

    label_dict     = config_params_dict.get( "label_dict", {} )
    msgs_dump_dir  = config_params_dict.get( "msgs_dump_dir", None )
    paint_function = config_params_dict.get( "paint_function", None )

    if label_dict is None:
        label_dict = fix_names( nodes_dict )

    all_nodes, all_topics, all_services = split_to_groups( nodes_dict )

    main_graph: Graph = generate_compact_graph( nodes_dict, show_services=True, labels_dict=label_dict )
    remove_ros_items( main_graph )
    if paint_function:
        paint_function( main_graph )

    nodes_subpages_dict    = generate_subpages_dict( nodes_dict, all_nodes, label_dict, 1, paint_function=paint_function )
    topics_subpages_dict   = generate_subpages_dict( nodes_dict, all_topics, label_dict, 0, paint_function=paint_function )
    services_subpages_dict = generate_subpages_dict( nodes_dict, all_services, label_dict, 0, paint_function=paint_function )

    topics_info = get_topics_info( nodes_dict, msgs_dump_dir )
    for topic_id, topic_data in topics_info.items():
        sub_dict = topics_subpages_dict[ topic_id ]
        sub_dict[ "msg_type" ]    = topic_data.get( "type", "" )
        sub_dict[ "msg_content" ] = topic_data.get( "content", "" )

    services_info = get_services_info( nodes_dict )
    for service_id, service_data in services_info.items():
        sub_dict = services_subpages_dict[ service_id ]
        sub_dict[ "svr_listener" ] = service_data.get( "listener", "" )
        sub_dict[ "msg_type" ]     = service_data.get( "type", "" )
        sub_dict[ "msg_content" ]  = service_data.get( "content", "" )

    sub_items = {}
    sub_items.update( nodes_subpages_dict )
    sub_items.update( topics_subpages_dict )
    sub_items.update( services_subpages_dict )

    params_dict = { "style": {},
                    "labels_dict": label_dict,
                    "main_page": { "graph": main_graph,
                                   "lists": generate_items_lists( all_nodes, all_topics, all_services )
                                   },
                    "sub_pages": sub_items
                    }

    generate_graph_html( out_dir, params_dict )


def generate_subpages_dict( nodes_dict, items_list, label_dict, neighbour_range, paint_function=None ):
    sub_items = {}
    for item_id in items_list:
        item_dict = {}
        sub_items[ item_id ] = item_dict
        item_graph: Graph = generate_full_graph( nodes_dict, labels_dict=label_dict )
        # item_graph: Graph = generate_full_graph( nodes_dict, labels_dict=label_dict, services_as_labels=False, services_as_nodes=True )
        preserve_neighbour_nodes( item_graph, [item_id], neighbour_range )
        if paint_function:
            paint_function( item_graph )
        
        graph_names = list( item_graph.getNodeNamesAll() )
        nodes_list  = sorted( filter_nodes( nodes_dict, graph_names ) )
        item_dict[ "graph" ]    = item_graph
        item_dict[ "msg_type" ]    = ""
        item_dict[ "msg_content" ] = ""

        topics_list   = sorted( filter_topics( nodes_dict, graph_names ) )
        services_list = sorted( get_services_from_dict( nodes_dict, [ item_id ] ) )
        
        item_dict[ "lists" ] = generate_items_lists( nodes_list, topics_list, services_list )
        
    return sub_items


def generate_items_lists( nodes_list, topics_list, services_list ):
    ret_list =  [ { "title": "ROS nodes",
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


def main():
    parser = argparse.ArgumentParser(description='rosnode flow graph')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--dump_dir', action='store', required=False, default="",
                         help="Dump directory containing 'rosnode' output data" )
    parser.add_argument( '--msgs_dump_dir', action='store', required=False, default="",
                         help="Dump directory containing 'rosmsg' output data" )
    parser.add_argument( '--srvs_dump_dir', action='store', required=False, default="",
                         help="Dump directory containing 'rosservice' output data" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help="Output HTML" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    nodes_dict = read_nodes( args.dump_dir )
    if len(nodes_dict) < 1:
        _LOGGER.warning( "no data found in %s", args.dump_dir )
        return

    label_dict = fix_names( nodes_dict )
    # info_dict  = get_node_info_dict( nodes_dict, label_dict, args.msgs_dump_dir, args.srvs_dump_dir )

    if len( args.outraw ) > 0 or len( args.outpng ) > 0:
        graph = generate_full_graph( nodes_dict )
        set_node_labels( graph, label_dict )
        if len( args.outraw ) > 0:
            graph.writeRAW( args.outraw )
        if len( args.outpng ) > 0:
            graph.writePNG( args.outpng )

    ##
    ## generate HTML data
    ##
    if args.outhtml and len( args.outdir ) > 0:
        config_params_dict = { "label_dict":    label_dict,
                               "msgs_dump_dir": args.msgs_dump_dir
                               }
        generate_pages( nodes_dict, args.outdir, config_params_dict )
