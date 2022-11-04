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

from rosdiagram.graph import Graph, get_nodes_names
from rosdiagram.io import read_file
from pydotplus.graphviz import Subgraph


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


## ===================================================================


def parse_content( content, build_deps=True ):
    deps_dict = {}
    
    package = None
    build_packages = True
    for line in content.splitlines():
        if len(line) < 1:
            continue

        if line[0] != ' ':
            ## new package
            package = line[:-1]
            deps_dict[ package ] = []
            continue

        if package is None:
            continue

        if "build_depend" in line:
            build_packages = True
            continue
        if "run_depend" in line:
            build_packages = False
            continue
        
        if build_packages != build_deps:
            continue

        if line.startswith( "  - " ):
            ## dependency
            deps_list = deps_dict[ package ]
            deps_list.append( line[4:] )
            continue
        
        _LOGGER.warning( "unhandled case: %s", line )

    return deps_dict


def generate_graph( deps_dict ):
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## generate main graph
    for key, vals in deps_dict.items():
        dot_graph.addNode( key, shape="box" )
        for dep in vals:
            dot_graph.addNode( dep, shape="box" )
            dot_graph.addEdge( key, dep )
    return dot_graph


def set_min_max_rank( dot_graph: Graph ):
    ## set nodes rank
    bottom_nodes = dot_graph.getNodesBottom()
    # print( "bottom:", get_nodes_names( bottom_nodes ) )
    dot_graph.setNodesRank( bottom_nodes, "max" )

    top_nodes = dot_graph.getNodesTop()
    # print( "top:", get_nodes_names( top_nodes ) )
    dot_graph.setNodesRank( top_nodes, "min" )
    

# def generate_graph( deps_dict, 
#                     include_filter_list=None, exclude_filter_list=None, filter_operator=None, 
#                     cluster_classifier=None, package_props=None ):
#     content = ""
#     
#     content += """digraph pgks_graph {
#   rankdir = LR;
#   #fontname="Helvetica,Arial,sans-serif"
#   #node [fontname="Helvetica,Arial,sans-serif"]
#   #edge [fontname="Helvetica,Arial,sans-serif"]
#   node [shape=box];
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


def generate( catkin_list_file ):
    content   = read_file( catkin_list_file )
    data_dict = parse_content( content, build_deps=False )
    graph     = generate_graph( data_dict )
    set_min_max_rank( graph )
    return graph
