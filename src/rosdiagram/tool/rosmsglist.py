# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse

from showgraph.io import prepare_filesystem_name

from rosdiagram.ros.rosmsgdata import read_msg_dir
from rosdiagram.ros.rossrvdata import read_srv_dir
from rosdiagram.graphviztohtml import generate_from_template


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

DATA_SUBDIR = "data"


## ===================================================================


def generate( messages_dir, services_dir, out_dir ):
    messages_dict = read_msg_dir( messages_dir )
    services_dict = read_srv_dir( services_dir )
    generate_pages( messages_dict, services_dict, out_dir )


def generate_pages( messages_dict, services_dict, out_dir ):
    template = "rosmsg.html"
    os.makedirs( out_dir, exist_ok=True )

    data_dir = os.path.join( out_dir, DATA_SUBDIR )
    os.makedirs( data_dir, exist_ok=True )

    messages_list = None
    services_list = None

    if messages_dict:
        messages_list = generate_subpages( messages_dict, out_dir )

    if services_dict:
        services_list = generate_subpages( services_dict, out_dir )

    main_dict = {   "style": {},
                    "item_type": None,
                    "item_content": None,
                    "msg_items": ("Messages", messages_list),
                    "srv_items": ("Services", services_list)
                    }
    generate_from_template( out_dir, main_dict, template_name=template )


def generate_subpages( messages_dict, out_dir ):
    template = "rosmsg.html"
    messages_list = []
    for item, content in messages_dict.items():
        item_file = prepare_filesystem_name( item )
        data_subdir  = os.path.join( DATA_SUBDIR, item_file )
        data_subpath = os.path.join( data_subdir, "main_page.html" )
        messages_list.append( (item, data_subpath) )

        data_fullpath = os.path.join( out_dir, data_subdir )
        os.makedirs( data_fullpath, exist_ok=True )
        data_dict = {   "style": {},
                        "item_type": item,
                        "item_content": content
                        }
        generate_from_template( data_fullpath, data_dict, template_name=template )
    return messages_list


## ===================================================================


def configure_parser( parser ):
    parser.description = 'rosmsg and rossrv messages list'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--msgsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosmsg' output" )
    parser.add_argument( '--srvsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rossrv' output" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    ##
    ## generate HTML data
    ##
    if len( args.outdir ) > 0:
        _LOGGER.info( "generating HTML" )
        generate( args.msgsdumppath, args.srvsdumppath, args.outdir )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
