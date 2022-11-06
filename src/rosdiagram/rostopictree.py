#!/usr/bin/env python3
#
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

import os

import logging
import argparse
import re 

from collections import defaultdict
from rosdiagram.io import read_file, read_list
from rosdiagram.graph import Graph


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


## ===================================================================


def read_topics( topics_dir ):
    topics_dict = {}
    topics_path = os.path.join( topics_dir, "list.txt" )
    topics_list = read_list( topics_path )
    for item in topics_list:
        topic_filename = item.replace( "/", "_" )
        topic_item_path = os.path.join( topics_dir, topic_filename + ".txt" )
        content   = read_dependencies( topic_item_path )
        deps_dict = parse_content( content )
        topics_dict[ item ] = deps_dict
    return topics_dict


def read_dependencies( deps_file=None ):
    content = ""
    if os.path.isfile( deps_file ):
        ## read content from file
        _LOGGER.info( "loading dependencies from file: %s", deps_file )
        with open( deps_file, 'r' ) as content_file:
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
    
    publishers_list  = False
    subscribers_list = False

    for line in content.splitlines():
        if len(line) < 1:
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
    deps_dict['pubs'] = publishers
    deps_dict['subs'] = subscribers
    return deps_dict


def match_node( line ):
    matched = re.findall( "^ \* (.*) \(.*$", line )
    m_size = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


## ===================================================================


def generate_graph( topics_dict ) -> Graph:
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    rename_topics_list = set()
    topics_list = list( topics_dict.keys() )
    for topic, lists in topics_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        
        pSize = len( pubs_list )
        for i in range(0, pSize):
            item = pubs_list[i]
            if topics_list.count( item ) > 0:
                pubs_list[i] = "n|" + item
                rename_topics_list.add( item )
                
        pSize = len( subs_list )
        for i in range(0, pSize):
            item = subs_list[i]
            if topics_list.count( item ) > 0:
                subs_list[i] = "n|" + item
                rename_topics_list.add( item )
            
            
    for topic in rename_topics_list:
        topics_dict[ "t|" + topic ] = topics_dict.pop( topic )

    ## add nodes
    for topic, lists in topics_dict.items():
        dot_graph.addNode( topic, shape="ellipse" )
        nodes: set = get_topic_nodes( lists )
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


def get_topic_nodes( topic_lists ) -> set:
    ret_nodes = set()
    pubs_list = topic_lists[ "pubs" ]
    subs_list = topic_lists[ "subs" ]
    ret_nodes.update( pubs_list )
    ret_nodes.update( subs_list )
    return ret_nodes


def generate( topic_info_dir ):
    data_dict = read_topics( topic_info_dir )
    graph     = generate_graph( data_dict )
    return graph
