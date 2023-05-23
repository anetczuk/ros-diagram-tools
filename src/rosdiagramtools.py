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
import rosdiagram.tool.packagetree as packagetree
import rosdiagram.tool.classifynodes as classifynodes
import rosdiagram.tool.buildtime as buildtime
import rosdiagram.tool.rosnodegraph as rosnodegraph
import rosdiagram.tool.rostopicgraph as rostopicgraph
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

    subparser = subparsers.add_parser('packagetree', help='catkin packages graph')
    subparser.set_defaults( func=packagetree.process_arguments )
    packagetree.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('classifynodes', help='match nodes to packages')
    subparser.set_defaults( func=classifynodes.process_arguments )
    classifynodes.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('buildtime', help='catkin build time')
    subparser.set_defaults( func=buildtime.process_arguments )
    buildtime.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosnodegraph', help='rosnode connection graph')
    subparser.set_defaults( func=rosnodegraph.process_arguments )
    rosnodegraph.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rostopicgraph', help='rostopic connection graph')
    subparser.set_defaults( func=rostopicgraph.process_arguments )
    rostopicgraph.configure_parser( subparser )

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
