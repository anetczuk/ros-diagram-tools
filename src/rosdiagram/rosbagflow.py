#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2022 Arkadiusz Netczuk <dev.arnet@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

try:
    ## following import success only when file is directly executed from command line
    ## otherwise will throw exception when executing as parameter for "python -m"
    # pylint: disable=W0611
    import __init__
except ImportError:
    ## when import fails then it means that the script was imported
    ## in this case __init__ is already loaded
    pass

# pylint: disable=C0413

import os
import sys
import logging


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


if __name__ == '__main__':
    ## allow having executable script inside package and have proper imports
    ## replace directory of main package (prevent inconsistent imports)
    sys.path[0] = os.path.join( SCRIPT_DIR, os.pardir )


from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_ros1
from rosbags.typesys import get_types_from_msg, register_types

from rosdiagram.rostopictree import read_topics, get_topic_subs_dict
from rosdiagram.plantuml import SequenceGraph, generate_seq_diagram


## ===================================================================


def generate( bag_path, topic_dump_dir, outdir, group_topics=True, group_subs=True, detect_loops=True ):
    topic_data = read_topics( topic_dump_dir )
    topic_subs = get_topic_subs_dict( topic_data )

    for _, subs in topic_subs.items():
        for name in subs.copy():
            if name in ( "/rosout" ):
                subs.remove( name )
            if name.startswith( "/rostopic_" ):
                subs.remove( name )
            if name.startswith( "/record_" ):
                subs.remove( name )
    
    # create reader instance and open for reading
    with Reader( bag_path ) as reader:
        print( "bag statistics:" )
        print( "total messages:", reader.message_count )
        print( "time span:", str( reader.duration / 1000000000 / 60 ) + "m" )

        ## reader.connections
        ## reader.messages
        ## reader.topics: List[ str ]

        ## Connection object:
        ## ['__add__', '__annotations__', '__class__', '__contains__', '__delattr__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__getitem__', '__getnewargs__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__iter__', '__le__', '__len__', '__lt__', '__module__', '__mul__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__rmul__', '__setattr__', '__sizeof__', '__slots__', '__str__', '__subclasshook__', '_asdict', '_field_defaults', '_field_types', '_fields', '_fields_defaults', '_make', '_replace', 'count', 'ext', 'id', 'index', 'md5sum', 'msgcount', 'msgdef', 'msgtype', 'owner', 'topic']
        ## Connection(id=0, topic='/fleet/environment/id', msgtype='std_msgs/msg/String', msgdef='string data\n', md5sum='992ce8a1687cec8c8bd883ec73ca41d1', msgcount=1, ext=ConnectionExtRosbag1(callerid='/vb_fleet', latching=1), owner=<rosbags.rosbag1.reader.Reader object at 0x7f6cd3fdd280>)

        # topic and msgtype information is available on .connections list
        print( "topics:" )
        for connection in reader.connections:
            print( connection.topic, connection.msgtype, connection.msgcount )

        ## generating sequence diagram
        seq_diagram = SequenceGraph()

        # iterate over messages
        ## iterates items in timestamp order
        print( "messages:" )
        messages = reader.messages()
        if not messages:
            print( "no message found" )
            return
        first_item = next( messages )
        first_timestamp = first_item[1]
        
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/mrs/zone_active_list':
                continue
#             if connection.topic == '/vb_fleet_manager/worker_info_list':
#                 continue
#             if connection.topic == '/mrs/visualization/robot_position':
#                 continue
#             if connection.topic == '/mrs/robot_info_list':
#                 continue
 
            subscribers = topic_subs[ connection.topic ]
            if not subscribers:
                ## topic without subscribers
                continue 

            ext = connection.ext
            topic_publisher = ext.callerid
 
            time_diff = timestamp - first_timestamp
            seq_diagram.addCallSeq( topic_publisher, subscribers, time_diff, connection.topic )
#             for sub in subscribers:
#                 seq_diagram.addCall( topic_publisher, sub, timestamp, connection.topic )
 
#             valid, msg = deserialize( rawdata, connection )
#             if valid is False:
#                 print( "unable to deserialize:", timestamp, connection.topic, connection.msgtype )
#                 continue
#  
#             print( "------------------ message ------------------ " )
#             print( timestamp, topic_publisher, connection.topic, connection.msgtype )
# #             print( msg )

        seq_diagram.process( group_topics, detect_loops )

        bag_name = os.path.basename( bag_path )
        out_path = os.path.join( outdir, f"flow_{bag_name}.puml" )
        generate_seq_diagram( seq_diagram, out_path, group_subs )


def deserialize( rawdata, connection ):
    ret_data = deserialize_raw( rawdata, connection.msgtype )
    if ret_data[0] is True:
        return ret_data

    msg_type = get_types_from_msg( connection.msgdef, connection.msgtype )
    register_types( msg_type )
    return deserialize_raw( rawdata, connection.msgtype )


def deserialize_raw( rawdata, msgtype ):
    try:
        msg = deserialize_ros1( rawdata, msgtype )
        return (True, msg)
    except KeyError:
        return (False, None)


## ===================================================================


def main():
    parser = argparse.ArgumentParser(description='catkin deps tree')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--topic_dump_dir', action='store', required=False, default="",
                         help="Dump directory containing 'rostopic' output data" )
    parser.add_argument( '--bag_path', action='store', required=False, default="",
                         help="Path to rosbag file" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )
    parser.add_argument( '--group_topics', action='store_true', help="Group multiple topics on one call" )
    parser.add_argument( '--group_subs', action='store_true', help="Group topic's subscribers in one UML group" )
    parser.add_argument( '--no-detect_loops', action='store_false', help="Detect message loops and group in one UML loop" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    generate( args.bag_path, args.topic_dump_dir, args.outdir, args.group_topics, args.group_subs, args.no_detect_loops )


if __name__ == '__main__':
    import argparse

    main()
