# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse
import yaml

from showgraph.io import read_dict, read_list
from rosdiagram.tool import classifynodes
from rosdiagram.tool import codedistribution
from rosdiagram.tool import rosparamlist
from rosdiagram.tool import packagetree
from rosdiagram.tool import rosnodegraph
from rosdiagram.tool import rosindex


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

DATA_SUBDIR = "data"


## ===================================================================


def configure_parser( parser ):
    parser.description = 'index of diagrams'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--dumprootdir', action='store', required=False, default="", help="Path directory with standard dump directories" )
    parser.add_argument( '--launchdumppath', action='store', required=False, default="",
                         help="Path fo directory containing dumped 'roslaunch' output" )
    parser.add_argument( '--classifynodesfile', action='store', required=False, default="",
                         help="Nodes classification input file" )
    parser.add_argument( '--descriptionjsonfile', action='store', required=False, default="", help="Path to JSON file with items description" )
    parser.add_argument( '--pkgsfilterlist', action='store', required=False, default="", help="PAth to file with list of packages to filter" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    if not args.dumprootdir:
        return
    if not args.outdir:
        return

    ##
    ## generate HTML data
    ##

    _LOGGER.info( "generating HTML output" )

    clocpacks_info_dir = os.path.join(args.dumprootdir, "clocpackinfo")
    params_info_file = os.path.join(args.dumprootdir, "paraminfo", "params.yml")
    packs_info_dir = os.path.join(args.dumprootdir, "packinfo")

    nodes_info_dir = os.path.join(args.dumprootdir, "nodeinfo")
    topics_info_dir = os.path.join(args.dumprootdir, "topicinfo")
    services_info_dir = os.path.join(args.dumprootdir, "serviceinfo")
    msgs_info_dir = os.path.join(args.dumprootdir, "msginfo")
    srv_info_dir = os.path.join(args.dumprootdir, "srvinfo")

    nodes_classify_dict = None
    if args.classifynodesfile and os.path.isfile( args.classifynodesfile ):
        _LOGGER.info( "reading nodes classify dictionary" )
        nodes_classify_dict = read_dict( args.classifynodesfile )
    if not nodes_classify_dict:
        _LOGGER.info( "reading nodes classify data" )
        nodes_classify_dict = classifynodes.classify_nodes( packs_info_dir, args.launchdumppath )

    description_dict = read_dict( args.descriptionjsonfile )

    index_items_list = []

    if os.path.isdir( clocpacks_info_dir ):
        _LOGGER.info( "generating codedistribution output" )
        clocpacks_out_dir = os.path.join(args.outdir, "clockpackview")
        os.makedirs( clocpacks_out_dir, exist_ok=True )
        clocpacks_out_file = os.path.join(clocpacks_out_dir, "graph.png")

        cloc_data_dict = codedistribution.read_cloc_data( clocdumpdir=clocpacks_info_dir, filteritemspath=args.pkgsfilterlist )
        cloc_graph = codedistribution.generate_graph( cloc_data_dict )
        cloc_graph.writePNG( clocpacks_out_file )

        index_items_list.append( ("code distribution graph", clocpacks_out_file ) )

    if os.path.isfile( params_info_file ):
        _LOGGER.info( "generating rosparamlist output" )
        params_out_dir = os.path.join(args.outdir, "paramview")

        with open( params_info_file, 'r', encoding='utf-8' ) as content_file:
            params_dict = yaml.safe_load( content_file )

        rosparamlist.generate_pages(params_dict, params_out_dir)

        params_out_file = os.path.join(params_out_dir, "main_page.html")
        index_items_list.append( ("parameters view", params_out_file ) )

    if os.path.isdir( packs_info_dir ):
        _LOGGER.info( "generating packagetree output" )
        packages_out_dir = os.path.join(args.outdir, "packageview")

        top_packages_list = read_list(args.pkgsfilterlist)

        packages_dict = packagetree.read_pack_data( packs_info_dir )

        config_params_dict = {  "top_list": top_packages_list,
                                "highlight_list": top_packages_list,
                                "nodes_classification": nodes_classify_dict,
                                "nodes_description": description_dict,
                                "paint_function": None
                                }
        packagetree.generate_pages( packages_dict, packages_out_dir, config_params_dict )

        packages_out_file = os.path.join(packages_out_dir, "full_graph.html")
        index_items_list.append( ("packages view", packages_out_file ) )

    if os.path.isdir( nodes_info_dir ):
        _LOGGER.info( "generating rosnodegraph output" )
        nodes_out_dir = os.path.join(args.outdir, "nodeview")
        nodes_dict, node_label_dict = rosnodegraph.read_nodes_data(nodes_info_dir, include_ros_internals=True)
        rosnodegraph.generate_node_pages( nodes_out_dir,
                                          nodes_dict,
                                          node_label_dict,
                                          nodes_classify_dict=nodes_classify_dict,
                                          topics_dump_dir=topics_info_dir,
                                          msgs_dump_dir=msgs_info_dir,
                                          services_dump_dir=services_info_dir,
                                          srvs_dump_dir=srv_info_dir,
                                          description_dict=description_dict
                                          #,
                                          # mainfullgraph=args.mainfullgraph,
                                          # highlight_list_file=args.highlightitems,
                                          # paint_function=None
                                          )

        nodes_out_file = os.path.join(nodes_out_dir, "full_graph.html")
        index_items_list.append( ("nodes view", nodes_out_file ) )

    if index_items_list:
        rosindex.generate_pages( index_items_list, args.outdir )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
