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

from rosdiagram.tool import codedistribution
from rosdiagram.tool import packagetree
from rosdiagram.tool import classifynodes
from rosdiagram.tool import buildtime
from rosdiagram.tool import rosparamlist
from rosdiagram.tool import rosnodegraph
from rosdiagram.tool import rostopicgraph
from rosdiagram.tool import rosbagflow
from rosdiagram.tool import rosverify


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## =============================================================


def main():
    parser = argparse.ArgumentParser(description='ROS diagram tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )

    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools",
                                        dest='tool', required=False )

    ## =================================================

    subparser = subparsers.add_parser('codedistribution', help='source code distribution over packages')
    subparser.set_defaults( func=codedistribution.process_arguments )
    codedistribution.configure_parser( subparser )

    ## =================================================

    subparser = subparsers.add_parser('packagetree', help='packages graph')
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

    subparser = subparsers.add_parser('rosparamlist', help='rosparam parameters list')
    subparser.set_defaults( func=rosparamlist.process_arguments )
    rosparamlist.configure_parser( subparser )

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
