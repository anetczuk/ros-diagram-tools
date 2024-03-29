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
from rosdiagram.ros.rostopicdata import read_topics
from rosdiagram.ros.rosservicedata import read_services
from rosdiagram.graphviztohtml import generate_from_template


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

DATA_SUBDIR = "data"


## ===================================================================


def generate( messages_dir, services_dir, out_dir, outhtml, outmarkdown, topicsdumppath=None, servicesdumppath=None ):
    msg_dict = read_msg_dir( messages_dir )
    srv_dict = read_srv_dir( services_dir )
    topics_dict = None
    if topicsdumppath:
        topics_dict = read_topics( topicsdumppath )
        use_dict = {}
        for topic, data in topics_dict.items():
            use_dict[ topic ] = data.get("type")
        topics_dict = use_dict
    services_dict = None
    if servicesdumppath:
        services_dict = read_services( servicesdumppath )
        use_dict = {}
        for service, data in services_dict.items():
            use_dict[ service ] = data.get("type")
        services_dict = use_dict
    generate_pages( msg_dict, srv_dict, out_dir, outhtml, outmarkdown, topics_dict, services_dict )


def generate_pages( msg_dict, srv_dict, out_dir, outhtml, outmarkdown, topics_dict, services_dict ):
    os.makedirs( out_dir, exist_ok=True )

    data_dir = os.path.join( out_dir, DATA_SUBDIR )
    os.makedirs( data_dir, exist_ok=True )

    messages_list = None
    services_list = None

    if msg_dict:
        messages_list = generate_subpages( msg_dict, out_dir, outhtml, outmarkdown, topics_dict )

    if srv_dict:
        services_list = generate_subpages( srv_dict, out_dir, outhtml, outmarkdown, services_dict )

    main_dict = {   "style": {},
                    "item_type": None,
                    "item_content": None,
                    "msg_items": ("Message defs", messages_list),
                    "srv_items": ("Service defs", services_list)
                    }

    if outhtml:
        template = "rosmsg.html"
        generate_from_template( out_dir, main_dict, template_name=template )

    if outmarkdown:
        template = "rosmsg.md"
        generate_from_template( out_dir, main_dict, template_name=template )


def generate_subpages( data_dict, out_dir, outhtml, outmarkdown, use_dict ):
    data_fullpath = os.path.join( out_dir, DATA_SUBDIR )
    os.makedirs( data_fullpath, exist_ok=True )

    messages_list = []
    for item, content in data_dict.items():
        ## extract users of items
        users_list = []
        if use_dict:
            for user_id, user_type in use_dict.items():
                if user_type == item:
                    users_list.append( user_id )

        item_file = prepare_filesystem_name( item )
        item_filename = f"{item_file}.autolink"
        data_subpath = os.path.join( DATA_SUBDIR, item_filename )
        in_use = len(users_list)
        messages_list.append( (item, data_subpath, in_use) )

        page_dict = {   "style": {},
                        "item_type": item,
                        "item_content": content,
                        "item_users": users_list
                        }

        if outhtml:
            template = "rosmsg.html"
            generate_from_template( data_fullpath, page_dict, template_name=template, page_filename=item_filename )

        if outmarkdown:
            template = "rosmsg.md"
            generate_from_template( data_fullpath, page_dict, template_name=template, page_filename=item_filename )

    return messages_list


## ===================================================================


def configure_parser( parser ):
    parser.description = 'rosmsg and rossrv messages list'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--msgsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosmsg' output" )
    parser.add_argument( '--srvsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rossrv' output" )
    parser.add_argument( '--topicsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rostopic' output" )
    parser.add_argument( '--servicesdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rostopic' output" )
    parser.add_argument( '--outhtml', action='store_true', help='Output HTML' )
    parser.add_argument( '--outmarkdown', action='store_true', help='Output Markdown' )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output directory" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    ##
    ## generate data
    ##
    if (args.outhtml or args.outmarkdown) and args.outdir:
        _LOGGER.info( "generating graphs" )
        generate( args.msgsdumppath, args.srvsdumppath, args.outdir, args.outhtml, args.outmarkdown,
                  args.topicsdumppath, args.servicesdumppath )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
