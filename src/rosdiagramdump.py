#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import sys
import os
import logging
import argparse

import base64

from rosdiagram import dumpscripts


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

TMP_DIR = "/tmp/rosdiagramdump"


## =============================================================


def extract_scripts( args ):
    out_dir = args.outdir
    extract_scripts_to( out_dir )


def extract_scripts_to( out_dir ):
    os.makedirs( out_dir, exist_ok=True )
    for script_data in dumpscripts.SCRIPTS_LIST:
        script_name = script_data[0]
        script_out_path = os.path.join( out_dir, script_name )
        _LOGGER.info( "extracting script: %s", script_out_path )
        extract_script( script_data, script_out_path )


def execute_script( script_data, args=None ):
    if args is None:
        args = []

    ## extract all scripts, because some scripts can depend on other scripts (e.g. dump_ros.sh)
    extract_scripts_to( TMP_DIR )

    script_name = script_data[0]
    script_out_path = os.path.join( TMP_DIR, script_name )

    args_string = " ".join( args )
    if os.system( f"{script_out_path} {args_string}" ) != 0:
        raise RuntimeError( f"failed executing script: {script_out_path}" )


def extract_script( script_data, script_out_path ):
    with open( script_out_path, "w", encoding='utf-8' ) as script:
        content = script_data[1]
        decoded_content = decode_content( content )
        script.write( decoded_content )
    if os.system( f"chmod +x '{script_out_path}'" ) != 0:
        raise RuntimeError( f"unable to set executable attribute to script: {script_out_path}" )


def decode_content( content ):
    decoded = base64.b64decode( content )
    return decoded.decode("utf-8")


## =============================================================


def dumpros( args ):
    args_list = []
    args_list.append( args.outdir )
    if not args.bashdump:
        args_list.append( "--fast" )
    execute_script( dumpscripts.DUMP_ROS_SH, args_list )


def dumprosrelative( args ):
    args_list = []
    args_list.append( args.outdir )
    if not args.bashdump:
        args_list.append( "--fast" )
    execute_script( dumpscripts.DUMP_ROSRELATIVE_SH, args_list )


def dumpclocdir( args ):
    args_list = []
    args_list.append( f"{args.clocrundir}" )
    args_list.append( f"{args.outdir}" )
    execute_script( dumpscripts.DUMP_CLOCDIR_SH, args_list )


def dumpclocpath( args ):
    args_list = []
    args_list.append( f"{args.packfile}" )
    args_list.append( f"{args.outdir}" )
    execute_script( dumpscripts.DUMP_CLOCPACK_SH, args_list )


def dumpcatkindeps( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_CATKINDEPS_SH, args_list )


def dumprosparam( args ):
    args_list = []
    args_list.append( args.launchfile )
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSPARAM_SH, args_list )


def dumproslaunch( args ):
    args_list = []
    args_list.append( args.launchfile )
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSLAUNCH_SH, args_list )


def dumprosmsg( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSMSG_SH, args_list )


def dumprosnode( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSNODE_SH, args_list )


def dumprospack( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSPACK_SH, args_list )


def dumprosservice( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSSERVICE_SH, args_list )


def dumprossrv( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSSRV_SH, args_list )


def dumprostopic( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROSTOPIC_SH, args_list )


## =============================================================


def main():                                                             # pylint: disable=R0915
    parser = argparse.ArgumentParser(description='dump tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )

    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools",
                                        dest='tool', required=False )

    ## =================================================

    description = "dump result of 'cloc' command on given directory"
    subparser = subparsers.add_parser('dumpclocdir', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumpclocdir )
    subparser.add_argument( '--clocrundir', action='store', required=True, default="",
                            help="Directory to analyze by 'cloc'" )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump result of 'cloc' command on pack data"
    subparser = subparsers.add_parser('dumpclocpack', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumpclocpath )
    subparser.add_argument( '--packfile', action='store', required=True, default="",
                            help="List file dumped with `dumprospack` tool" )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump catkin dependencies of packages in workspace (from package.xml)"
    subparser = subparsers.add_parser('dumpcatkindeps', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumpcatkindeps )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump data from 'rosparam'"
    subparser = subparsers.add_parser('dumprosparam', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprosparam )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump data from 'rospack'"
    subparser = subparsers.add_parser('dumprospack', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprospack )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump messages info"
    subparser = subparsers.add_parser('dumprosmsg', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprosmsg )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump services definitions"
    subparser = subparsers.add_parser('dumprossrv', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprossrv )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump nodes info"
    subparser = subparsers.add_parser('dumprosnode', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprosnode )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump topics info"
    subparser = subparsers.add_parser('dumprostopic', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprostopic )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump services info"
    subparser = subparsers.add_parser('dumprosservice', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprosservice )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump node names of launch file"
    subparser = subparsers.add_parser('dumproslaunch', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumproslaunch )
    subparser.add_argument( '--launchfile', action='store', required=True, default="", help="Launch file" )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump majority of data"
    subparser = subparsers.add_parser('dumpros', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumpros )
    subparser.add_argument( '--bashdump', action='store_true', required=False, default="",
                            help="Use bash dump instead of Python (slower, but more stable)" )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "dump majority of data (related msgs and srvs)"
    subparser = subparsers.add_parser('dumprosrelative', help=description)
    subparser.description = description
    subparser.set_defaults( func=dumprosrelative )
    subparser.add_argument( '--bashdump', action='store_true', required=False, default="",
                            help="Use bash dump instead of Python (slower, but more stable)" )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    description = "extract embedded scripts to files"
    subparser = subparsers.add_parser('extractscripts', help=description)
    subparser.description = description
    subparser.set_defaults( func=extract_scripts )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

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
