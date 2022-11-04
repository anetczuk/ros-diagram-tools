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
from rosdiagram.io import read_file
from rosdiagram.graph import Graph


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


## ===================================================================


def filter_operator( deps_dict: dict ) -> dict:
    return None


def filter_operator( deps_dict: dict ) -> dict:
    return None


## ===================================================================


def read_topics( topics_dir ):
    topics_dict = {}
    topics_path = os.path.join( topics_dir, "list.txt" )
    topics_list = load_list( topics_path )
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


def generate_graph( topics_dict ):
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )

    topics_list = topics_dict.keys()

    ## add nodes
    for topic, lists in topics_dict.items():
        dot_graph.addNode( topic, shape="ellipse" )
        nodes: set = get_topic_nodes( lists )
        nodes.difference_update( topics_list )
        for item in nodes:
            dot_graph.addNode( item, shape="box" )
    
#     print( dot_graph.toString() )

    ## add edges    
    for topic, lists in topics_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        for pub in pubs_list:
            dot_graph.addEdge( pub, topic )
        for sub in subs_list:
            dot_graph.addEdge( topic, sub )

    return dot_graph


#     content = ""
# 
#     content += """digraph pgks_graph {
# #  rankdir = LR;
#   #fontname="Helvetica,Arial,sans-serif"
#   #node [fontname="Helvetica,Arial,sans-serif"]
#   #edge [fontname="Helvetica,Arial,sans-serif"]
# #  node [shape=box];
# 
# #  overlap=false;
# #  esep=10.0;
# #  nodesep=10.0;
# #  len=100.0;
# #  weight = 0.01;
# #  sep=100;
# 
# """
#     
#     topics_list = topics_dict.keys()
# 
#     filter_nodes = [ "/vb_fleet_planner" ]
#     
#     neighbour_nodes  = set()
#     neighbour_topics = set()
# 
#     # nodes_dict  = get_nodes_dict( topics_dict )
#     
#     for f_node in filter_nodes:
#         for topic, lists in topics_dict.items():
#             nodes: set = get_topic_nodes( lists )
#             nodes.difference_update( topics_list )          ## remove topics from nodes
#             if f_node in nodes:
#                 neighbour_topics.add( topic )
#     
#     for f_node in filter_nodes:
#         for topic, lists in topics_dict.items():
#             if topic in neighbour_topics:
#                 nodes: set = get_topic_nodes( lists )
#                 nodes.difference_update( topics_list )      ## remove topics from nodes
#                 neighbour_nodes.update( nodes )
# 
#     ## remove filtered topics
#     for topic, lists in topics_dict.copy().items():
#         if topic not in neighbour_topics:
#             del topics_dict[ topic ]
# 
#     content +=  "\n## nodes\n"
#     for topic, lists in topics_dict.items():
#         content +=  "  \"%s\" [shape=ellipse]\n" % ( topic )
#         nodes: set = get_topic_nodes( lists )
#         nodes.difference_update( topics_list )
#         for item in nodes:
#             if item in filter_nodes:
#                 content +=  "  \"%s\" [shape=box, style=filled, fillcolor=yellow]\n" % ( item )
#             else:
#                 content +=  "  \"%s\" [shape=box]\n" % ( item )
# 
#     content +=  "\n## edges\n"
#     for topic, lists in topics_dict.items():
#         pubs_list = lists[ "pubs" ]
#         subs_list = lists[ "subs" ]
#         for pub in pubs_list:
#             content +=  "  \"%s\" -> \"%s\"\n" % ( pub, topic )
#         for sub in subs_list:
#             content +=  "  \"%s\" -> \"%s\"\n" % ( topic, sub )
# 
#     # for topic, lists in topics_dict.items():
#     #     pubs_list = lists[ "pubs" ]
#     #     subs_list = lists[ "subs" ]
#     #     for pub in pubs_list:
#     #         for sub in subs_list:
#     #             content +=  "  \"%s\" -> \"%s\"\n" % ( pub, sub )
# 
#     content += "}\n"
#     return content


def get_topic_nodes( topic_lists ) -> set:
    ret_nodes = set()
    pubs_list = topic_lists[ "pubs" ]
    subs_list = topic_lists[ "subs" ]
    ret_nodes.update( pubs_list )
    ret_nodes.update( subs_list )
    return ret_nodes


def get_nodes_dict( topics_dict ):
    nodes_map = {}
    
    topics_list = topics_dict.keys()
    for topic, lists in topics_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        for pub_node in pubs_list:
            if pub_node not in topics_list:
                node_dict  = get_create_item( nodes_map, pub_node, {} )
                items_list = get_create_item( node_dict, "pubs", [] )
                items_list.append( topic )
        for sub_node in subs_list:
            if sub_node not in topics_list:
                node_dict  = get_create_item( nodes_map, sub_node, {} )
                items_list = get_create_item( node_dict, "subs", [] )
                items_list.append( topic )

    return nodes_map

def get_create_item( dict_obj, key, default_val ):
    if key not in dict_obj:
        dict_obj[ key ] = default_val
    return dict_obj[ key ]


#
#     filtered = filter_packages( deps_dict, include_filter_list, exclude_filter_list, filter_operator )
#     if filtered is not None:
#         deps_dict = filtered
#
#     packages_list = flat_list( deps_dict )
#
#     clusters_dict = {}
#     if cluster_classifier is not None:
#         clusters_dict = cluster_classifier( packages_list )
#         if clusters_dict is None:
#             clusters_dict = {}
#
#     pkgs_props_dict = {}
#     if package_props is not None:
#         pkgs_props_dict = package_props( packages_list )
#         if pkgs_props_dict is None:
#             pkgs_props_dict = {}
#
#     ## generate subgraphs
# #     print( "clusters:\n", clusters_dict )
#     for key, vals in clusters_dict.items():
#         content +=  "  subgraph cluster_%s{\n" % key
#         content +=  "    label = \"%s\";\n" % key
#         content +=  "    style = \"dashed\";\n"
#         for dep in vals:
#             content +=  "    \"%s\"\n" % dep
#         content +=  "  }\n"
#
#     ## generate main graph
#     all_deps = set()
#     for key, vals in deps_dict.items():
# #         if len( vals ) < 1:
# #             ## package without dependencies
# #             content +=  "  \"%s\"\n" % key
# #             continue
#
#         if key in pkgs_props_dict:
#             pks_props  = pkgs_props_dict[ key ]
#             node_props = inline_props( pks_props )
#             if node_props is None:
#                 node_props = ""
#             content +=  "  \"%s\" %s\n" % (key, node_props)
#
#         ## package with additional deps
#         all_deps.update( vals )
#         for dep in vals:
#             content +=  "  \"%s\" -> \"%s\"\n" % (key, dep)
#
#     ## set ranks
#     top_list    = set()
#     for key in deps_dict:
#         if key not in all_deps:
#             top_list.add( key )
#
#     bottom_list = set()
#     for key, vals in deps_dict.items():
#         if len( vals ) < 1:
#             bottom_list.add( key )
#             continue
#     dict_keys = deps_dict.keys()
#     for dep in all_deps:
#         if dep not in dict_keys:
#             bottom_list.add( dep )
#     bottom_list = [x for x in bottom_list if x not in top_list]
#     bottom_list = set( bottom_list )
#
#     clustered_pckgs = set()
#     for key, vals in clusters_dict.items():
#         clustered_pckgs.update( vals )
#
#     top_list    -= clustered_pckgs
#     bottom_list -= clustered_pckgs
#
#     if len(top_list) > 0:
#         content += "  { rank=min; \"" + "\"; \"".join(top_list) + "\"; }\n"
#     if len(bottom_list) > 0:
#         content += "  { rank=max; \"" + "\"; \"".join(bottom_list) + "\"; }\n"
#
#     content += "}\n"
#     return content


def flat_list( deps_dict ):
    ret_list = []
    for key, vals in deps_dict.copy().items():
        if key not in ret_list:
            ret_list.append( key )
        for dep in vals.copy():
            if dep not in ret_list:
                ret_list.append( dep )
    return ret_list


def top_set( deps_dict ):
    top_set = set( deps_dict.keys() )
    for _, vals in deps_dict.items():
        for dep in vals:
            if dep in top_set:
                top_set.remove( dep )
    return top_set


def filter_packages( deps_dict, include_filter_list=None, exclude_filter_list=None, filter_operator=None ):
    if include_filter_list is not None:
        ## include items
        for key, vals in deps_dict.copy().items():
            if key not in include_filter_list:
                del deps_dict[ key ]
                continue
            for dep in vals.copy():
                if dep not in include_filter_list:
                    vals.remove( dep )

    if exclude_filter_list is not None:
        ## exclude items
        for key, vals in deps_dict.copy().items():
            if key in exclude_filter_list:
                del deps_dict[ key ]
                continue
            for dep in vals.copy():
                if dep in exclude_filter_list:
                    vals.remove( dep )

    if filter_operator is not None:
        deps_dict = filter_operator( deps_dict )

    return deps_dict


def inline_props( node_props ):
#     return None
    if len(node_props)< 1:
        return None
    content = "["
    for key, val in node_props.items():
        content += "%s=%s," % ( key, val )
    content += "]"
    return content


def load_list( file_path ):
    if len(file_path) < 1:
        return None
    ret_list = []
    with open( file_path, 'r' ) as content_file:
        for line in content_file:
            ret_list.append( line.strip() )
    return ret_list


def read_data( topic_info_dir ):
    pass


def generate( topic_info_dir ):
    data_dict = read_topics( topic_info_dir )
    graph     = generate_graph( data_dict )
    return graph
