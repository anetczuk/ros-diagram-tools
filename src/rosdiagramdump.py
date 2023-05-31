#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import sys, os
import logging
import argparse

import base64

import rosdiagram.dumpscripts as dumpscripts

# import rosdiagram.tool.codedistribution as codedistribution
# import rosdiagram.tool.packagexmltree as packagexmltree
# import rosdiagram.tool.classifynodes as classifynodes
# import rosdiagram.tool.buildtime as buildtime
# import rosdiagram.tool.rosnodegraph as rosnodegraph
# import rosdiagram.tool.rostopicgraph as rostopicgraph
# import rosdiagram.tool.rosbagflow as rosbagflow
# import rosdiagram.tool.rosverify as rosverify


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## =============================================================


def extract_scripts( args ):
    out_dir = args.outdir
    os.makedirs( out_dir, exist_ok=True )
    for script_data in dumpscripts.SCRIPTS_LIST:
        script_name = script_data[0]
        script_out_path = os.path.join( out_dir, script_name )
        _LOGGER.info( "extracting script: %s", script_out_path )
        extract_script( script_data, script_out_path )


def execute_script( script_data, args=None ):
    if args is None:
        args = []

    script_name = script_data[0]
    script_out_path = os.path.join( "/tmp", script_name )
    extract_script( script_data, script_out_path )

    os.system( f"chmod +x '{script_out_path}'" )
    args_string = " ".join( args )
    os.system( f"{script_out_path} {args_string}" )


def extract_script( script_data, script_out_path ):
    with open( script_out_path, "w") as script:
        content = script_data[1]
        decoded_content = decode_content( content )
        script.write( decoded_content )


def decode_content( content ):
    decoded = base64.b64decode( content )
    return decoded.decode("utf-8")


## =============================================================


def dumpros( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_ROS_SH, args_list )


def dumpclocdir( args ):
    args_list = []
    args_list.append( f"--clocrundir {args.clocrundir}" )
    args_list.append( f"--outfile {args.outfile}" )
    execute_script( dumpscripts.DUMP_CLOC_PY, args_list )


def dumpcatkindeps( args ):
    args_list = []
    args_list.append( args.outdir )
    execute_script( dumpscripts.DUMP_CATKIN_SH, args_list )


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


def main():
    parser = argparse.ArgumentParser(description='ROS diagram tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )
    
    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools", dest='tool', required=False )
    
    ## =================================================
    
    subparser = subparsers.add_parser('extractscripts', help="extract embedded scripts to files")
    subparser.set_defaults( func=extract_scripts )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    subparser = subparsers.add_parser('dumpros', help="dump majority of data")
    subparser.set_defaults( func=dumpros )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================
    
    subparser = subparsers.add_parser('dumpclocdir', help="dump result of 'cloc' command on given directory")
    subparser.set_defaults( func=dumpclocdir )
    subparser.add_argument( '--clocrundir', action='store', required=True, default="",
                            help="Directory to analyze by 'cloc'" )
    subparser.add_argument( '--outfile', action='store', required=True, default="", help="Output file" )

    ## =================================================

    subparser = subparsers.add_parser('dumpcatkindeps', help="dump catkin build deps of workspace packages")
    subparser.set_defaults( func=dumpcatkindeps )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )

    ## =================================================

    subparser = subparsers.add_parser('dumproslaunch', help="dump node names of launch file")
    subparser.set_defaults( func=dumproslaunch )
    subparser.add_argument( '--launchfile', action='store', required=True, default="", help="Launch file" )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )
    
    ## =================================================

    subparser = subparsers.add_parser('dumprosmsg', help="dump messages info")
    subparser.set_defaults( func=dumprosmsg )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )
    
    ## =================================================

    subparser = subparsers.add_parser('dumprosnode', help="dump nodes info")
    subparser.set_defaults( func=dumprosnode )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )
    
    ## =================================================

    subparser = subparsers.add_parser('dumprospack', help="dump data from 'rospack'")
    subparser.set_defaults( func=dumprospack )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )
    
    ## =================================================

    subparser = subparsers.add_parser('dumprosservice', help="dump services info")
    subparser.set_defaults( func=dumprosservice )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )
    
    ## =================================================

    subparser = subparsers.add_parser('dumprossrv', help="dump services definitions")
    subparser.set_defaults( func=dumprossrv )
    subparser.add_argument( '--outdir', action='store', required=True, default="", help="Output directory" )
    
    ## =================================================

    subparser = subparsers.add_parser('dumprostopic', help="dump topics info")
    subparser.set_defaults( func=dumprostopic )
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
