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

import rosdiagram.tool.codedistribution as codedistribution
import rosdiagram.tool.catkintree as catkintree
import rosdiagram.tool.classifynodes as classifynodes
import rosdiagram.tool.catkinschedule as catkinschedule
import rosdiagram.tool.rosnodetree as rosnodetree
import rosdiagram.tool.rostopictree as rostopictree
import rosdiagram.tool.rosbagflow as rosbagflow
import rosdiagram.tool.rosverify as rosverify


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## =============================================================


def main():
    parser = argparse.ArgumentParser(description='ROS diagram tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )

    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools", dest='tool', required=False )

    ## =================================================

    subparser = subparsers.add_parser('codedistribution', help='source code distribution over packages')
    subparser.set_defaults( func=codedistribution.process_arguments )
    codedistribution.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('catkintree', help='catkin packages graph')
    subparser.set_defaults( func=catkintree.process_arguments )
    catkintree.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('classifynodes', help='match nodes to packages')
    subparser.set_defaults( func=classifynodes.process_arguments )
    classifynodes.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('catkinschedule', help='catkin build schedule')
    subparser.set_defaults( func=catkinschedule.process_arguments )
    catkinschedule.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosnodetree', help='rosnode connection graph')
    subparser.set_defaults( func=rosnodetree.process_arguments )
    rosnodetree.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rostopictree', help='rostopic connection graph')
    subparser.set_defaults( func=rostopictree.process_arguments )
    rostopictree.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosbagflow', help='generate sequence diagram based on messages from rosbag')
    subparser.set_defaults( func=rosbagflow.process_arguments )
    rosbagflow.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosverify', help='verify ROS packages')
    subparser.set_defaults( func=rosverify.process_arguments )
    rosverify.configure_parser( subparser )

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