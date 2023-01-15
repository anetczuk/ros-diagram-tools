# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import re
from typing import List
import copy

from rosdiagram.io import read_list, prepare_filesystem_name
from rosdiagram.utils import get_create_item


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_topics( topic_dir ):
    """ Returns dict with following structure:
        { "<topic_id>": {                   ## topic id
                          "pubs": [],       ## list of publishers of topic
                          "subs": []        ## list of subscribers of topic
                         }
          }
    """
    topics_dict = {}
    topics_path = os.path.join( topic_dir, "list.txt" )
    _LOGGER.debug( "reading topics list file: %s", topics_path )
    topics_list = read_list( topics_path )
    for item in topics_list:
        topic_filename  = prepare_filesystem_name( item )
        topic_item_path = os.path.join( topic_dir, topic_filename + ".txt" )
        content   = read_dependencies( topic_item_path )
        deps_dict = parse_content( content )
        topics_dict[ item ] = deps_dict
    return topics_dict


def read_dependencies( deps_file=None ):
    content = ""
    if os.path.isfile( deps_file ):
        ## read content from file
        _LOGGER.debug( "reading topic info file: %s", deps_file )
        with open( deps_file, 'r', encoding='utf-8' ) as content_file:
            content = content_file.read()
    else:
        ## execute 'catkin list'
        #TODO: implement
        _LOGGER.error( "executing catkin not implemented" )
        content = ""
    return content


def parse_content( content ):
    publishers  = []
    subscribers = []

    msg_type         = None
    publishers_list  = False
    subscribers_list = False

    for line in content.splitlines():
        if len(line) < 1:
            continue

        if "Type:" in line:
            msg_type = match_type( line )
            continue

        if "Publishers:" in line:
            publishers_list  = True
            subscribers_list = False
            continue
        if "Subscribers:" in line:
            publishers_list  = False
            subscribers_list = True
            continue

        if publishers_list is True:
            if subscribers_list is True:
                print( "forbidden state" )
                continue
            node = match_node( line )
            if node is None:
                continue
            publishers.append( node )
        else:
            if subscribers_list is False:
                ## both false
                continue
            node = match_node( line )
            if node is None:
                continue
            subscribers.append( node )

    # print( publishers, subscribers )

    deps_dict = {}
    deps_dict['type'] = msg_type
    deps_dict['pubs'] = publishers
    deps_dict['subs'] = subscribers
    return deps_dict


def match_node( line ):
    matched = re.findall( r"^ \* (.*) \(.*$", line )
    m_size  = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


def match_type( line ):
    matched = re.findall( r"^Type: (.*)$", line )
    m_size  = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


def get_topic_subs_dict( topic_data ):
    ret_pubs = {}
    ret_subs = {}
    for topic, lists_dict in topic_data.items():
        pubs: List[str] = lists_dict[ "pubs" ]
        subs: List[str] = lists_dict[ "subs" ]
        ret_pubs[ topic ] = pubs
        ret_subs[ topic ] = subs
    return ret_subs


## ===================================================================


## it happens that topic and node has the same name, so it has to be prefixed
def fix_names( topics_dict ):
    label_dict = {}
    all_topics = list( topics_dict.keys() )

    for topic in all_topics:
        item_id = "t_" + topic
        topics_dict[ item_id ] = topics_dict.pop( topic )
        label_dict[ item_id ] = topic

    for _, lists in topics_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]

        for node in pubs_list.copy():
            item_id = "n_" + node
            pubs_list.append( item_id )
            pubs_list.remove( node )
            label_dict[ item_id ] = node

        for node in subs_list.copy():
            item_id = "n_" + node
            subs_list.append( item_id )
            subs_list.remove( node )
            label_dict[ item_id ] = node

    return label_dict


def get_nodes( topic_lists ) -> List[ str ]:
    ret_nodes: List[ str ] = []
    pubs_list = topic_lists[ "pubs" ]
    subs_list = topic_lists[ "subs" ]
    ret_nodes.extend( pubs_list )
    ret_nodes.extend( subs_list )
    return list( dict.fromkeys(ret_nodes) )


def get_nodes_all( topics_dict ) -> List[ str ]:
    ret_set = []
    for _, lists in topics_dict.items():
        nodes: list = get_nodes( lists )
        ret_set.extend( nodes )
    return list( dict.fromkeys(ret_set) )


def get_topic_type( topics_dict, topic_id ) -> str:
    topic_data = topics_dict.get( topic_id, {} )
    return topic_data.get( "type", None )


def common_topics( data1_dict, data2_dict ):
    data1_topics = set()
    for topic in data1_dict:
        data1_topics.add( topic )
    data2_topics = set()
    for topic in data2_dict:
        data2_topics.add( topic )
    return data1_topics.intersection( data2_topics )


def preserve_common_topics( data1_dict, data2_dict ):
    common_set = common_topics( data1_dict, data2_dict )

    for key in data1_dict.copy():
        if key not in common_set:
            del data1_dict[ key ]

    for key in data2_dict.copy():
        if key not in common_set:
            del data2_dict[ key ]

    return common_set


def join_data_dicts( data1_dict, data2_dict ):
    ret_dict = copy.deepcopy( data1_dict )

    for topic, items in data2_dict.items():
        ret_lists = get_create_item( ret_dict, topic, [] )

        ret_pubs  = get_create_item( ret_lists, "pubs", [] )
        from_pubs = get_create_item( items, "pubs", [] )
        for item in from_pubs:
            if item not in ret_pubs:
                ret_pubs.append( item )

        ret_subs  = get_create_item( ret_lists, "subs", [] )
        from_subs = get_create_item( items, "subs", [] )
        for item in from_subs:
            if item not in ret_subs:
                ret_subs.append( item )

    return ret_dict


def join_common_topics( data1_dict, data2_dict ):
    copy1_dict = copy.deepcopy( data1_dict )
    copy2_dict = copy.deepcopy( data2_dict )
    preserve_common_topics( copy1_dict, copy2_dict )
    return join_data_dicts( copy1_dict, copy2_dict )
