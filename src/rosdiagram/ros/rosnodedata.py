# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import re
from typing import List, Dict

from rosdiagram.io import read_list, prepare_filesystem_name, read_file
from rosdiagram.utils import get_create_item
from rosdiagram.ros.rostopicdata import read_topics, get_topic_type
from rosdiagram.ros.rosmsgdata import read_msg
from rosdiagram.ros.rosservicedata import read_service, get_service_type
from rosdiagram.ros.rossrvdata import read_srv


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


class ROSNodeData():
    
    def __init__(self, nodes_dict ):
        self.nodes_dict       = nodes_dict
        self.nodes_label_dict = None

    def fixNames(self):
        self.nodes_label_dict  = fix_names( self.nodes_dict )


## ===================================================================


def read_nodes( nodes_dir ):
    nodes_dict = {}
    nodes_path = os.path.join( nodes_dir, "list.txt" )
    _LOGGER.debug( "reading nodes list file: %s", nodes_path )
    topics_list = read_list( nodes_path )
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
        _LOGGER.debug( "reading node info file: %s", deps_file )
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
    all_nodes = list( nodes_dict.keys() )

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


## return dictionary containing service and topic ID and as value HTML to display
def get_node_info_dict( nodes_dict, label_dict, msgs_dump_dir, srvs_dump_dir ) -> Dict[ str, str ]:
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


## ==============================================================


def filter_nodes( nodes_dict, names_list ):
    return [ name for name in names_list if name in nodes_dict ]


def filter_topics( nodes_dict, names_list ):
    all_topics = get_topics_all( nodes_dict )
    return [ name for name in names_list if name in all_topics ]


def get_topics_all( nodes_dict ):
    ret_list: List[ str ] = []
    for _, node_list in nodes_dict.items():
        topics_list = get_topics( node_list )
        ret_list.extend( topics_list )
    return ret_list


def get_topics( node_lists ) -> List[ str ]:
    ret_set: List[ str ] = []
    pubs_list = node_lists[ "pubs" ]
    subs_list = node_lists[ "subs" ]
    pubs_list = get_names_from_list( pubs_list )
    subs_list = get_names_from_list( subs_list )
    ret_set.extend( pubs_list )
    ret_set.extend( subs_list )
    return list( dict.fromkeys(ret_set) )


def get_topics_from_dict( nodes_dict, nodes_list ) -> List[ str ]:
    ret_list = []
    for node_id in nodes_list:
        topics_list = get_topics( nodes_dict.get( node_id, {} ) )
        ret_list.extend( topics_list )
    return ret_list


def get_topics_info( nodes_dict, topics_dict, msgs_dump_dir=None ):
    ret_data = {}
    for _, node_list in nodes_dict.items():
        pubs_list = node_list[ "pubs" ]
        for item_id, item_type in pubs_list:
            #topic_type = get_topic_type( topics_dict, item_id )
            topic_data = topics_dict.get( item_id, {} )
            topic_type = topic_data.get( "type", None )
            if topic_type:
                item_type = topic_type
            topic_data = {}
            ret_data[ item_id ] = topic_data
            topic_data["pubs"] = topic_data.get("pubs", None)
            topic_data["subs"] = topic_data.get("subs", None)
            topic_data["type"] = item_type
            
            msg_content = read_msg( msgs_dump_dir, item_type )
            topic_data["content"] = msg_content

        subs_list = node_list[ "subs" ]
        for item_id, item_type in subs_list:
            #topic_type = get_topic_type( topics_dict, item_id )
            topic_data = topics_dict.get( item_id, {} )
            topic_type = topic_data.get( "type", None )
            if topic_type:
                item_type = topic_type
            topic_data = {}
            ret_data[ item_id ] = topic_data
            topic_data["pubs"] = topic_data.get("pubs", None)
            topic_data["subs"] = topic_data.get("subs", None)
            topic_data["type"] = item_type

            msg_content = ""
            if msgs_dump_dir:
                msg_file    = prepare_filesystem_name( item_type )
                msg_path    = os.path.join( msgs_dump_dir, msg_file + ".txt" )
                msg_content = read_file( msg_path )
            topic_data["content"] = msg_content

    return ret_data


def get_services( node_lists ) -> List[ str ]:
    servs_list = node_lists.get( "servs", [] )
    return get_names_from_list( servs_list )


def get_services_from_dict( nodes_dict, nodes_list ) -> List[ str ]:
    ret_list = []
    for node_id in nodes_list:
        servs_list = get_services( nodes_dict.get( node_id, {} ) )
        ret_list.extend( servs_list )
    return ret_list


def get_services_info( nodes_dict, services_dict, srvs_dump_dir ):
    ret_data = {}
    for node_id, node_list in nodes_dict.items():
        srvs_list = node_list[ "servs" ]
        for item_id, item_type in srvs_list:
            service_type = get_service_type( services_dict, item_id )
            if service_type:
                item_type = service_type
            srv_data = {}
            ret_data[ item_id ] = srv_data
            srv_data["listener"] = node_id
            srv_data["type"] = item_type
            
            srv_content = read_srv( srvs_dump_dir, item_type )
            srv_data["content"] = srv_content

    return ret_data


def get_names_from_list( items_list ) -> List[ str ]:
    ret_list = [ item[0] for item in items_list ]
    return list( dict.fromkeys( ret_list ) )


## =====================================================


def split_to_groups( nodes_dict ):
    all_nodes    = set( nodes_dict.keys() )
    all_topics   = set()
    all_services = set()
    for _, lists in nodes_dict.items():
        topics: set = get_topics( lists )
        all_topics.update( topics )
        services: set = get_services( lists )
        all_services.update( services )

    all_nodes    = sorted( list( all_nodes ) )
    all_topics   = sorted( list( all_topics ) )
    all_services = sorted( list( all_services ) )

    return [ all_nodes, all_topics, all_services ]
