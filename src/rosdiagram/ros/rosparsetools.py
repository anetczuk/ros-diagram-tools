#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

#
# Keep this file as independent as possible (without dependencies), because it will be run on
# ROS devices to retrieve ROS data.
#

import sys
import os
import logging

import re
from typing import List

import argparse


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def extract_topics_from_node_data(node_info_dict):
    pubs_list = node_info_dict[ "pubs" ]
    subs_list = node_info_dict[ "subs" ]
    pubs_list = get_names_from_list( pubs_list )
    subs_list = get_names_from_list( subs_list )

    ret_list = set()
    ret_list.update( pubs_list )
    ret_list.update( subs_list )
    return ret_list


def extract_services_from_node_data(node_info_dict):
    servs_list = node_info_dict[ "servs" ]
    servs_list = get_names_from_list( servs_list )
    return set( servs_list )


def read_nodes( nodes_dir ):
    if not nodes_dir:
        return None
    nodes_dict = {}
    nodes_path = os.path.join( nodes_dir, "list.txt" )
    _LOGGER.debug( "reading nodes list file: %s", nodes_path )
    nodes_list = read_list( nodes_path )
    for item in nodes_list:
        node_filename = prepare_filesystem_name( item )
        node_item_path = os.path.join( nodes_dir, node_filename + ".txt" )
        deps_dict = parse_node_info_file( node_item_path )
        nodes_dict[ item ] = deps_dict
    return nodes_dict


def parse_node_info_file( info_file ):
    content = read_file( info_file )
    if not content:
        return None
    return parse_node_info( content )


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
            node = node_match_topic( line )
            if node is None:
                continue
            publications.append( node )
        elif section_mode == 2:
            ## subs
            node = node_match_topic( line )
            if node is None:
                continue
            subscribtions.append( node )
        elif section_mode == 3:
            ## servs
            node = node_match_service( line )
            if node is None:
                continue
            services.append( (node, None) )
        else:
            _LOGGER.warning( "forbidden state %s", section_mode )
            continue

    deps_dict = {}
    deps_dict["pubs"]  = publications
    deps_dict["subs"]  = subscribtions
    deps_dict["servs"] = services
    return deps_dict


## return pair: (topic name, message type)
def node_match_topic( line ):
    matched = re.findall( r"^ \* (\S+)\s*\[(.*)\]\s*$", line )
#     matched = re.findall( r"^ \* (\S+)\s*[.*]\s*$", line )
    if len( matched ) != 1 and len( matched[0] ) != 2:
        _LOGGER.warning( "invalid state for line: %s %s", line, matched )
        return None
    match_data = matched[0]
    return ( match_data[0], match_data[1] )


def node_match_service( line ):
    matched = re.findall( r"^ \* (\S+).*$", line )
    m_size = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s %s", line, m_size )
        return None
    return matched[0]


## ===================================================================


def extract_topic_messages( topics_dict ):
    ret_set = set()
    for _, topic_data in topics_dict.items():
        msg_type = topic_data["type"]
        ret_set.add( msg_type )
    return ret_set


def read_topics( topic_dir ):
    """Read topics dump directory.

    Returns dict with following structure:
    { "<topic_id>": {                   ## topic id
                      "type": str,      ## topic type - type of message
                      "pubs": [],       ## list of publishers of topic
                      "subs": []        ## list of subscribers of topic
                     }
      }
    """
    if not topic_dir:
        return None
    topics_dict = {}
    topics_path = os.path.join( topic_dir, "list.txt" )
    _LOGGER.debug( "reading topics list file: %s", topics_path )
    topics_list = read_list( topics_path )
    for item in topics_list:
        topic_filename  = prepare_filesystem_name( item )
        topic_item_path = os.path.join( topic_dir, topic_filename + ".txt" )
        deps_dict = parse_topic_info_file( topic_item_path )
        topics_dict[ item ] = deps_dict
    return topics_dict


def parse_topic_info_file( info_file ):
    content = read_file( info_file )
    if not content:
        return None
    return parse_topic_info( content )


def parse_topic_info( content ):
    """Parse topic dump content.

    Returns dict: { type: topic_type
                    pubs: publishers_list
                    subs: subscribers_list }
    """
    publishers  = []
    subscribers = []

    msg_type         = None
    publishers_list  = False
    subscribers_list = False

    for line in content.splitlines():
        if len(line) < 1:
            continue

        if "Type:" in line:
            msg_type = topic_match_type( line )
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
                _LOGGER.warning( "forbidden state" )
                continue
            node = topic_match_node( line )
            if node is None:
                continue
            publishers.append( node )
        else:
            if subscribers_list is False:
                ## both false
                continue
            node = topic_match_node( line )
            if node is None:
                continue
            subscribers.append( node )

    deps_dict = {}
    deps_dict['type'] = msg_type
    deps_dict['pubs'] = publishers
    deps_dict['subs'] = subscribers
    return deps_dict


def topic_match_node( line ):
    matched = re.findall( r"^ \* (.*) \(.*$", line )
    m_size  = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


def topic_match_type( line ):
    matched = re.findall( r"^Type: (.*)$", line )
    m_size  = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


## ===================================================================


def extract_service_messages( services_dict ):
    ret_set = set()
    for _, service_data in services_dict.items():
        msg_type = service_data["type"]
        ret_set.add( msg_type )
    return ret_set


def read_services( service_dir ):
    """
    Read services from dump directory.

    Return dict with following structure:
    { "<service_id>": {                   ## service id
                        "type": str,      ## service type - type of message
                       }
      }
    """
    if not service_dir:
        return None
    services_dict = {}
    list_path = os.path.join( service_dir, "list.txt" )
    _LOGGER.debug( "reading services list file: %s", list_path )
    topics_list = read_list( list_path )
    for item in topics_list:
        services_dict[ item ] = read_service( service_dir, item )
    return services_dict


def read_service( services_dump_dir, service_id ):
    if not services_dump_dir or not service_id:
        return None
    item_file = prepare_filesystem_name( service_id )
    item_path = os.path.join( services_dump_dir, item_file + ".txt" )
    if not os.path.isfile( item_path ):
        return None
    item_content = read_file( item_path )
    return parse_service_info( item_content )


def parse_service_info( content ):
    msg_type = None

    for line in content.splitlines():
        if len(line) < 1:
            continue

        if "Type:" in line:
            msg_type = service_match_type( line )
            continue

    deps_dict = {}
    deps_dict['type'] = msg_type
    return deps_dict


def service_match_type( line ):
    matched = re.findall( r"^Type: (.*)$", line )
    m_size  = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


## ===================================================================


def get_names_from_list( items_list ) -> List[ str ]:
    ret_list = [ item[0] for item in items_list ]
    return list( dict.fromkeys( ret_list ) )


def read_list( file_path ):
    if not os.path.isfile( file_path ):
        return []
    ret_list = []
    with open( file_path, 'r', encoding='utf-8' ) as content_file:
        for line in content_file:
            ret_list.append( line.strip() )
    return ret_list


def write_list( file_path, dump_list ):
    with open(file_path, 'w', encoding='utf8' ) as fp:
        fp.write('\n'.join(dump_list))


## read content from file
def read_file( file_path=None ):
    if not os.path.isfile( file_path ):
        return None
    _LOGGER.debug( "loading content from file: %s", file_path )
    with open( file_path, 'r', encoding='utf-8' ) as content_file:
        content = content_file.read()
    return content


def prepare_filesystem_name( name ):
    new_name = name
    new_name = new_name.replace( "/", "_" )
    new_name = new_name.replace( "|", "_" )
    new_name = new_name.replace( "-", "_" )
    return new_name


## =====================================================
## =====================================================


def dump_rosnode( out_dir ):
    os.makedirs(out_dir, exist_ok=True)
    dump_script = os.path.join( SCRIPT_DIR, "dump_rosnode.sh" )
    if os.system( f"{dump_script} {out_dir}" ) != 0:
        sys.exit(1)


def dump_rostopic( topic_list, out_dir ):
    os.makedirs(out_dir, exist_ok=True)
    out_list_path = os.path.join( out_dir, "list.txt" )
    write_list( out_list_path, topic_list )
    dump_script = os.path.join( SCRIPT_DIR, "dump_rostopic.sh" )
    if os.system( f"{dump_script} {out_dir} --listprovided" ) != 0:
        sys.exit(1)


def dump_rosservice( service_list, out_dir ):
    os.makedirs(out_dir, exist_ok=True)
    out_list_path = os.path.join( out_dir, "list.txt" )
    write_list( out_list_path, service_list )
    dump_script = os.path.join( SCRIPT_DIR, "dump_rosservice.sh" )
    if os.system( f"{dump_script} {out_dir} --listprovided" ) != 0:
        sys.exit(1)


def dump_rosmsg( msg_list, out_dir ):
    os.makedirs(out_dir, exist_ok=True)
    out_list_path = os.path.join( out_dir, "list.txt" )
    write_list( out_list_path, msg_list )
    dump_script = os.path.join( SCRIPT_DIR, "dump_rosmsg.sh" )
    if os.system( f"{dump_script} {out_dir} --listprovided" ) != 0:
        sys.exit(1)


def dump_rossrv( srv_list, out_dir ):
    os.makedirs(out_dir, exist_ok=True)
    out_list_path = os.path.join( out_dir, "list.txt" )
    write_list( out_list_path, srv_list )
    dump_script = os.path.join( SCRIPT_DIR, "dump_rossrv.sh" )
    if os.system( f"{dump_script} {out_dir} --listprovided" ) != 0:
        sys.exit(1)


## =====================================================


def extractmsgs_configure( parser ):
    parser.description = 'extract msgs types from topics dump dir'
    parser.add_argument( '--topicsdumpdir', action='store', required=True, default="",
                         help="Path to dump of rostopic" )
    parser.add_argument( '--outlist', action='store', required=False, default="",
                         help="Path output list" )


def extractmsgs_process( args ):
    data_dir = args.topicsdumpdir
    out_list_path = args.outlist

    if out_list_path:
        _LOGGER.info("extracting msgs from %s to %s", data_dir, out_list_path)
    else:
        _LOGGER.info("extracting msgs from %s", data_dir)

    out_dir = os.path.dirname( out_list_path )
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    data_dict = read_topics( data_dir )
    messages_list = extract_topic_messages( data_dict )
    if out_list_path:
        write_list( out_list_path, messages_list )
    else:
        print( "\n".join(messages_list) )


## =====================================================


def extractsrvs_configure( parser ):
    parser.description = 'extract srvs types from services dump dir'
    parser.add_argument( '--servicesdumpdir', action='store', required=True, default="",
                         help="Path to dump of rosservice" )
    parser.add_argument( '--outlist', action='store', required=False, default="",
                         help="Path output list" )


def extractsrvs_process( args ):
    data_dir = args.servicesdumpdir
    out_list_path = args.outlist

    if out_list_path:
        _LOGGER.info("extracting srvs from %s to %s", data_dir, out_list_path)
    else:
        _LOGGER.info("extracting srvs from %s", data_dir)

    out_dir = os.path.dirname( out_list_path )
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    data_dict = read_services( data_dir )
    messages_list = extract_service_messages( data_dict )
    if out_list_path:
        write_list( out_list_path, messages_list )
    else:
        print( "\n".join(messages_list) )


## =====================================================


def main():
    parser = argparse.ArgumentParser(description='ROS parse tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )

    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools",
                                        dest='tool', required=False )

    ## =================================================

    subparser = subparsers.add_parser('extractmsgs', help='extract msgs types from topics dump dir')
    subparser.set_defaults( func=extractmsgs_process )
    extractmsgs_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('extractsrvs', help='extract srvs types from services dump dir')
    subparser.set_defaults( func=extractsrvs_process )
    extractsrvs_configure( subparser )

    ## =================================================

    args = parser.parse_args()

    if args.listtools is True:
        tools_list = list( subparsers.choices.keys() )
        print( ", ".join( tools_list ) )
        return

    if "func" not in args:
        ## no command given -- print help message
        parser.print_help()
        sys.exit( 1 )
        return

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    args.func( args )


if __name__ == '__main__':
    main()
