# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse

from rosdiagram.graphviztohtml import generate_from_template


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

DATA_SUBDIR = "data"


## ===================================================================


def generate_pages( items_list, out_dir ):
    os.makedirs( out_dir, exist_ok=True )

    ## convert to relative path
    converted_items_list = []
    for title, path in items_list:
        # out_dir
        rel_path = os.path.relpath(path, out_dir)
        converted_items_list.append( (title, rel_path) )

    main_dict = {   "style": {},
                    "items_list": converted_items_list
                    }
    template = "rosindex.html"
    generate_from_template( out_dir, main_dict, template_name=template )


## ===================================================================


def configure_parser( parser ):
    parser.description = 'index of diagrams'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--packagesview', action='store', required=False, default="", help="Path to packages view" )
    parser.add_argument( '--paramsview', action='store', required=False, default="", help="Path to params view" )
    parser.add_argument( '--nodesview', action='store', required=False, default="", help="Path to nodes view" )
    parser.add_argument( '--topicsview', action='store', required=False, default="", help="Path to topics view" )
    parser.add_argument( '--customlist', action='store', required=False, default="", nargs='*',
                         help="Space-separated list of titles and links" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    items_list = []
    if args.packagesview:
        items_list.append( ( "packages view", args.packagesview ) )
    if args.paramsview:
        items_list.append( ( "parameters view", args.paramsview ) )
    if args.nodesview:
        items_list.append( ( "nodes view", args.nodesview ) )
    if args.topicsview:
        items_list.append( ( "topics view", args.topicsview ) )

    if args.customlist:
        customlist = args.customlist
        view_len = len(customlist)
        for index in range(1, view_len, 2):
            title = customlist[ index - 1 ]
            link  = customlist[ index ]
            items_list.append( (title, link) )
        if view_len % 2 == 1:
            items_list.append( (customlist[-1], "") )

    ##
    ## generate HTML data
    ##
    if len( args.outdir ) > 0:
        _LOGGER.info( "generating HTML graph" )
        generate_pages( items_list, args.outdir )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
