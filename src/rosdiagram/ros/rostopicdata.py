# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from typing import List
import copy

from rosdiagram.utils import get_create_item
from rosdiagram.ros.rosparsetools import read_topics                                # noqa pylint: disable=W0611
from rosdiagram.ros.rosutils import is_ros_internal_topic, is_ros_internal_node


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def get_topic_subs_dict( topic_data ):
    # ret_pubs = {}
    ret_subs = {}
    for topic, lists_dict in topic_data.items():
        # pubs: List[str] = lists_dict[ "pubs" ]
        subs: List[str] = lists_dict[ "subs" ]
        # ret_pubs[ topic ] = pubs
        ret_subs[ topic ] = subs
    return ret_subs


## ===================================================================


def filter_ros_topics_dict( topics_dict ):
    for topic_name, lists in topics_dict.copy().items():
        if is_ros_internal_topic( topic_name ):
            del topics_dict[ topic_name ]
            continue

        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]

        for node_name in pubs_list.copy():
            if is_ros_internal_node( node_name ):
                pubs_list.remove( node_name )

        for node_name in subs_list.copy():
            if is_ros_internal_node( node_name ):
                subs_list.remove( node_name )


NODE_PREFIX = "n_"
TOPIC_PREFIX = "t_"


## it happens that topic and node has the same name, so it has to be prefixed
def fix_names( topics_dict ):
    label_dict = {}
    all_topics = list( topics_dict.keys() )

    for topic in all_topics:
        item_id = TOPIC_PREFIX + topic
        topics_dict[ item_id ] = topics_dict.pop( topic )
        label_dict[ item_id ] = topic

    for _, lists in topics_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]

        for node in pubs_list.copy():
            item_id = NODE_PREFIX + node
            pubs_list.append( item_id )
            pubs_list.remove( node )
            label_dict[ item_id ] = node

        for node in subs_list.copy():
            item_id = NODE_PREFIX + node
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
