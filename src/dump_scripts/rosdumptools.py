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
import argparse

import rospkg
import rosnode
import rostopic
import rosservice
import rosmsg


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## =====================================================


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


def prepare_filesystem_name( name ):
    new_name = name
    new_name = new_name.replace( "/", "_" )
    new_name = new_name.replace( "|", "_" )
    new_name = new_name.replace( "-", "_" )
    return new_name


## =====================================================


def rospack_configure( parser ):
    parser.description = 'dump rospack data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rospack_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    rp = rospkg.RosPack()
    packages = rp.list()
    packages = sorted( packages )

    out_list = os.path.join( out_dir, "list.txt" )

    with open(out_list, 'w', encoding='utf8' ) as fp:
        for item in packages:
            pkg_path = rp.get_path(item)
            fp.write( f'{item} {pkg_path}\n' )

            depends1 = rp.get_depends(item, implicit=False)
            out_pkg = os.path.join( out_dir, f"{item}.txt" )
            write_list( out_pkg, depends1 )

    print( "Done." )


## =====================================================


def rosnode_configure( parser ):
    parser.description = 'dump rosnode data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rosnode_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    nodes_list = rosnode.get_node_names()
    nodes_list = sorted( nodes_list )

    out_list = os.path.join( out_dir, "list.txt" )

    write_list( out_list, nodes_list )

    for item in nodes_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            node_info = rosnode.get_node_info_description(item)
            fp.write( node_info )

    print( "Done." )


## =====================================================


def rostopic_configure( parser ):
    parser.description = 'dump rostopic data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rostopic_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    publishers, subscribers = rostopic.get_topic_list()
    topics_list = set()
    topics_list.update( [ pub[0] for pub in publishers ] )
    topics_list.update( [ sub[0] for sub in subscribers ] )
    topics_list = sorted( list( topics_list ) )

    out_list = os.path.join( out_dir, "list.txt" )

    write_list( out_list, topics_list )

    for item in topics_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            topic_info = rostopic.get_info_text( item )
            fp.write( topic_info )

    print( "Done." )


## =====================================================


def rosservice_configure( parser ):
    parser.description = 'dump rosservice data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rosservice_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    services_list = rosservice.get_service_list()
    services_list = sorted( list(services_list) )

    out_list = os.path.join( out_dir, "list.txt" )

    write_list( out_list, services_list )

    for item in services_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        service_node = rosservice.get_service_node( item )
        service_uri = rosservice.get_service_uri( item )
        service_type = rosservice.get_service_type( item )
        service_args = rosservice.get_service_args( item )

        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            out_data = f"""\
Node: {service_node}
URI: {service_uri}
Type: {service_type}
Args: {service_args}
"""
            fp.write( out_data )

    print( "Done." )


## =====================================================


def rosmsg_configure( parser ):
    parser.description = 'dump rosmsg data'
    parser.add_argument( '--listprovided', action='store_true', required=False, default="",
                         help="Read list of messages from list file" )
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rosmsg_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    out_list = os.path.join( out_dir, "list.txt" )

    print( f"Dumping data to {out_dir}" )

    rp = rospkg.RosPack()
    packages = rp.list()
    packages = sorted( packages )

    messages_list = set()
    if args.listprovided:
        # read items from list
        messages_list = read_list( out_list )
    else:
        for pkg in packages:
            msg_list = rosmsg.list_msgs( pkg, rp )
            if msg_list:
                messages_list.update( msg_list )
        messages_list = sorted( list( messages_list ) )
        write_list( out_list, messages_list )

    for item in messages_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        msg_info = rosmsg.get_msg_text( item, rospack=rp )

        print( f"Writing {out_info_path}" )
        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            fp.write( msg_info )

    print( "Done." )


## =====================================================


def rossrv_configure( parser ):
    parser.description = 'dump rossrv data'
    parser.add_argument( '--listprovided', action='store_true', required=False, default="",
                         help="Read list of messages from list file" )
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rossrv_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    out_list = os.path.join( out_dir, "list.txt" )

    print( f"Dumping data to {out_dir}" )

    rp = rospkg.RosPack()
    packages = rp.list()
    packages = sorted( packages )

    services_list = set()
    if args.listprovided:
        # read items from list
        services_list = read_list( out_list )
    else:
        for pkg in packages:
            srv_list = rosmsg.list_srvs( pkg, rp )
            if srv_list:
                services_list.update( srv_list )
        services_list = sorted( list( services_list ) )
        write_list( out_list, services_list )

    for item in services_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        srv_info = rosmsg.get_srv_text( item, rospack=rp )

        print( f"Writing {out_info_path}" )
        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            fp.write( srv_info )

    print( "Done." )


## =====================================================


def main():
    parser = argparse.ArgumentParser(description='ROS parse tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )

    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools",
                                        dest='tool', required=False )

    ## =================================================

    subparser = subparsers.add_parser('rospack', help='dump rospack data')
    subparser.set_defaults( func=rospack_process )
    rospack_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosnode', help='dump rosnode data')
    subparser.set_defaults( func=rosnode_process )
    rosnode_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rostopic', help='dump rostopic data')
    subparser.set_defaults( func=rostopic_process )
    rostopic_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosservice', help='dump rosservice data')
    subparser.set_defaults( func=rosservice_process )
    rosservice_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosmsg', help='dump rosmsg data')
    subparser.set_defaults( func=rosmsg_process )
    rosmsg_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rossrv', help='dump rossrv data')
    subparser.set_defaults( func=rossrv_process )
    rossrv_configure( subparser )

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
