# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree. 
#

# pylint: disable=C0413

import os
import sys
import logging

import re
from typing import Set
import argparse

from rosdiagram.htmlgenerator import generate_graph_html
from rosdiagram.graphviz import Graph, unquote_name_list, set_node_labels
from rosdiagram.io import read_list, prepare_filesystem_name, read_file
from rosdiagram.utils import get_create_item


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_nodes( nodes_dir ):
    nodes_dict = {}
    topics_path = os.path.join( nodes_dir, "list.txt" )
    topics_list = read_list( topics_path )
    for item in topics_list:
        node_filename = prepare_filesystem_name( item )
        node_item_path = os.path.join( nodes_dir, node_filename + ".txt" )
        content   = get_node_info( node_item_path )
        deps_dict = parse_node_info( content )
        nodes_dict[ item ] = deps_dict
    return nodes_dict


def get_node_info( deps_file=None ):
    content = ""
    if os.path.isfile( deps_file ):
        ## read content from file
        _LOGGER.debug( "loading dependencies from file: %s", deps_file )
        with open( deps_file, 'r', encoding='utf-8' ) as content_file:
            content = content_file.read()
    else:
        ## execute 'catkin list'
        #TODO: implement
        _LOGGER.error( "executing catkin not implemented" )
        content = ""
    return content


def parse_node_info( content ):
    publications  = []
    subscribtions = []
    services      = []

    ##  0 -- unset
    ##  1 -- pubs
    ##  2 -- subs
    ##  3 -- services
    section_mode = 0

    for line in content.splitlines():
        if len(line) < 1:
            section_mode = 0
            continue

        if "Publications:" in line:
            section_mode = 1
            continue
        if "Subscriptions:" in line:
            section_mode = 2
            continue
        if "Services:" in line:
            section_mode = 3
            continue

        if section_mode == 0:
            ## initial state
            continue
        if section_mode == 1:
            ## pubs
            node = match_topic( line )
            if node is None:
                continue
            publications.append( node )
        elif section_mode == 2:
            ## subs
            node = match_topic( line )
            if node is None:
                continue
            subscribtions.append( node )
        elif section_mode == 3:
            ## servs
            node = match_service( line )
            if node is None:
                continue
            services.append( (node, None) )
        else:
            print( "forbidden state", section_mode )
            continue

    # print( publishers, subscribers )

    deps_dict = {}
    deps_dict["pubs"]  = publications
    deps_dict["subs"]  = subscribtions
    deps_dict["servs"] = services
    return deps_dict


def match_topic( line ):
    matched = re.findall( r"^ \* (\S+)\s*\[(.*)\]\s*$", line )
#     matched = re.findall( r"^ \* (\S+)\s*[.*]\s*$", line )
    if len( matched ) != 1 and len( matched[0] ) != 2:
        _LOGGER.warning( "invalid state for line: %s %s", line, matched )
        return None
    match_data = matched[0]
    return ( match_data[0], match_data[1] )


def match_service( line ):
    matched = re.findall( r"^ \* (\S+).*$", line )
    m_size = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s %s", line, m_size )
        return None
    return matched[0]


## ===================================================================


def generate_full_graph( nodes_dict ) -> Graph:
    dot_graph = Graph()
    dot_graph.setName( "nodes_graph" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for node, lists in nodes_dict.items():
        dot_graph.addNode( node, shape="box" )

        topics: set = get_topics( lists )
        for item in topics:
            dot_graph.addNode( item, shape="ellipse" )

        services: set = get_services( lists )
        for item in services:
            dot_graph.addNode( item, shape="hexagon" )

    ## add edges
    for node, lists in nodes_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]
        servs_list = lists[ "servs" ]
        pubs_list  = get_names_from_list( pubs_list )
        subs_list  = get_names_from_list( subs_list )
        servs_list = get_names_from_list( servs_list )
        for pub in pubs_list:
            dot_graph.addEdge( node, pub )
        for sub in subs_list:
            dot_graph.addEdge( sub, node )
        for serv in servs_list:
            dot_graph.addEdge( serv, node )

    return dot_graph


def generate_main_graph( nodes_dict, show_services=True, labels_dict=None ):
    if labels_dict is None:
        labels_dict = {}
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for node_name, lists in nodes_dict.items():
        node_label = labels_dict.get( node_name, node_name )
        node_obj   = dot_graph.addNode( node_name, shape="box", label=node_label )
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


def create_topics_dict( nodes_dict ):
    topics_dict = {}
    for node, lists in nodes_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        pubs_list = get_names_from_list( pubs_list )
        subs_list = get_names_from_list( subs_list )
        for pub in pubs_list:
            topic_lists = get_create_item( topics_dict, pub, {} )
            items_list = get_create_item( topic_lists, "pubs", [] )
            get_create_item( topic_lists, "subs", [] )
            items_list.append( node )
        for sub in subs_list:
            topic_lists = get_create_item( topics_dict, sub, {} )
            get_create_item( topic_lists, "pubs", [] )
            items_list = get_create_item( topic_lists, "subs", [] )
            items_list.append( node )
    return topics_dict


## it happens that topic and node has the same name, so it has to be prefixed
def fix_names( nodes_dict ):
    label_dict = {}
    all_nodes = set( nodes_dict.keys() )

    for node in all_nodes:
        item_id = "n_" + node
        nodes_dict[ item_id ] = nodes_dict.pop( node )
        label_dict[ item_id ] = node

    for node, lists in nodes_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]
        servs_list = lists[ "servs" ]

        for topic_pair in pubs_list.copy():
            topic = topic_pair[0]
            item_id = "t_" + topic
            pubs_list.append( (item_id, topic_pair[1]) )
            pubs_list.remove( topic_pair )
            label_dict[ item_id ] = topic

        for topic_pair in subs_list.copy():
            topic = topic_pair[0]
            item_id = "t_" + topic
            subs_list.append( (item_id, topic_pair[1]) )
            subs_list.remove( topic_pair )
            label_dict[ item_id ] = topic

        for service_pair in servs_list.copy():
            service = service_pair[0]
            item_id = "s_" + service
            servs_list.append( (item_id, service_pair[1]) )
            servs_list.remove( service_pair )
            label_dict[ item_id ] = service

    return label_dict


def get_node_info_dict( nodes_dict, label_dict, msgs_dump_dir, srvs_dump_dir ):
    info_dict = {}

    for _, lists in nodes_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]
        servs_list = lists[ "servs" ]

        all_topics = pubs_list + subs_list
        for topic_pair in all_topics.copy():
            topic         = topic_pair[0]
            message_name  = topic_pair[1]
            content_file  = prepare_filesystem_name( message_name )
            content_path  = os.path.join( msgs_dump_dir, content_file + ".txt" )
            item_info     = prepare_code_content( message_name, content_path )
            info_dict[ topic ] = item_info

        for service_pair in servs_list.copy():
            service      = service_pair[0]
            service_name = label_dict.get( service, None )
            if service_name is None:
                continue
            content_file = prepare_filesystem_name( service_name )
            content_path = os.path.join( srvs_dump_dir, content_file + ".txt" )
            item_info    = prepare_code_content( service_name, content_path )
            info_dict[ service ] = item_info

    return info_dict


def prepare_code_content( code_title, code_path ):
    file_content = read_file( code_path )

    code_content = ""
    if file_content is None:
        code_content = f"""Message: <code>{code_title}</code>
"""
    else:
        file_content = file_content.strip()
        code_content = f"""\
Message: <code>{code_title}</code><br/>
<pre><code>{file_content}</code></pre>
"""
    return code_content


def get_topics( node_lists ) -> Set[ str ]:
    ret_set: Set[ str ] = set()
    pubs_list = node_lists[ "pubs" ]
    subs_list = node_lists[ "subs" ]
    pubs_list = get_names_from_list( pubs_list )
    subs_list = get_names_from_list( subs_list )
    ret_set.update( pubs_list )
    ret_set.update( subs_list )
    return ret_set


def get_names_from_list( items_list ) -> Set[ str ]:
    return set( item[0] for item in items_list )


def get_services( node_lists ) -> Set[ str ]:
    servs_list = node_lists[ "servs" ]
    return get_names_from_list( servs_list )


def split_to_groups( nodes_dict ):
    all_nodes    = set( nodes_dict.keys() )
    all_topics   = set()
    all_services = set()
    for _, lists in nodes_dict.items():
        topics: set = get_topics( lists )
        all_topics.update( topics )
        services: set = get_services( lists )
        all_services.update( services )

    all_nodes    = list( all_nodes )
    all_topics   = list( all_topics )
    all_services = list( all_services )

    return [ all_nodes, all_topics, all_services ]


def generate( node_info_dir ):
    data_dict = read_nodes( node_info_dir )
    graph     = generate_full_graph( data_dict )
    return graph


def remove_ros_items( graph: Graph ):
    all_names = graph.getNodeNamesAll()
    unquoted_names = unquote_name_list( all_names )
    for name in unquoted_names:
        if name in ( "/rosout", "n_/rosout", "t_/rosout" ):
            graph.removeNode( name )
        if name.startswith( "/rostopic_" ) or name.startswith( "n_/rostopic_" ):
            graph.removeNode( name )
        if name.startswith( "/record_" ) or name.startswith( "n_/record_" ):
            graph.removeNode( name )


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='rosnode flow graph')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--dump_dir', action='store', required=False, default="",
                         help="Dump directory containing 'rostopic' output data" )
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
    info_dict  = get_node_info_dict( nodes_dict, label_dict, args.msgs_dump_dir, args.srvs_dump_dir )

    if len( args.outraw ) > 0 or len( args.outpng ) > 0:
        graph = generate_full_graph( nodes_dict )
        set_node_labels( graph, label_dict )
        if len( args.outraw ) > 0:
            graph.writeRAW( args.outraw )
        if len( args.outpng ) > 0:
            graph.writePNG( args.outpng )

    data_groups  = split_to_groups( nodes_dict )
    nodes_groups = [ { "title": "ROS nodes",
                       "items": data_groups[0],
                       "neighbours_range": 1
                       },
                     { "title": "ROS topics",
                       "items": data_groups[1],
                       "neighbours_range": 0
                       },
                     { "title": "ROS services",
                       "items": data_groups[2],
                       "neighbours_range": 0
                       }
                     ]

    if args.outhtml and len( args.outdir ) > 0:
        main_graph = generate_main_graph( nodes_dict, show_services=True, labels_dict=label_dict )
        remove_ros_items( main_graph )

        params_dict = { "graph_factory": lambda: generate_full_graph( nodes_dict ),
                        "main_graph": main_graph,
                        "label_dict": label_dict,
                        "groups": nodes_groups,
                        "graph_info_dict": info_dict,
                        "neighbours_range": 1
                        }
        generate_graph_html( args.outdir, params_dict )
