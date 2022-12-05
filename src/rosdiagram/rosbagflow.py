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
from _collections import OrderedDict

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
import pprint
import json


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


if __name__ == '__main__':
    ## allow having executable script inside package and have proper imports
    ## replace directory of main package (prevent inconsistent imports)
    sys.path[0] = os.path.join( SCRIPT_DIR, os.pardir )


from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_ros1
from rosbags.typesys import get_types_from_msg, register_types

from rosdiagram.io import write_file
from rosdiagram.rostopictree import read_topics, get_topic_subs_dict
from rosdiagram.plantuml import SequenceGraph, generate_seq_diagram


## ===================================================================


def generate( bag_path, topic_dump_dir, outdir, group_topics=True, group_subs=True, detect_loops=True, write_messages=True ):
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

        # topic and msgtype information is available on .connections list

#         print( "topics:" )
#         for connection in reader.connections:
#             print( connection.topic, connection.msgtype, connection.msgcount )

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
#             if connection.topic == '/namespace1/topic1':
#                 continue
 
            subscribers = topic_subs[ connection.topic ]
            if not subscribers:
                ## topic without subscribers
                continue 

            ext = connection.ext
            topic_publisher = ext.callerid

            time_diff = timestamp - first_timestamp
            graph_item = seq_diagram.addCallSubs( topic_publisher, subscribers, time_diff, connection.topic )

            valid, msg = deserialize_msg( rawdata, connection )
            if valid is False:
                print( "unable to deserialize:", timestamp, connection.topic, connection.msgtype )
            else:
                graph_item.setMessageData( connection.msgtype, connection.msgdef, msg )

        seq_diagram.process( group_topics, detect_loops )

        bag_name = os.path.basename( bag_path )
        svg_path = f"flow_{bag_name}.svg"
        out_path = os.path.join( outdir, "full_graph.html" )
        write_main_page( svg_path, out_path )

        ## generating message pages
        if write_messages:
            out_dir = os.path.join( outdir, "msg" )
            os.makedirs( out_dir, exist_ok=True )
            loops = seq_diagram.getLoops()
            for loop in loops:
                if loop.repeats > 1:
                    pass
                for item in loop.items:
                    out_name = f"{item.index}_msg.html"
                    item.setProp( "url", "msg/" + out_name )
                    out_path = os.path.join( out_dir, out_name )
                    write_message_page( item, out_path )

        ## write diagram
        out_path = os.path.join( outdir, f"flow_{bag_name}.puml" )
        generate_seq_diagram( seq_diagram, out_path, group_subs )


def deserialize_msg( rawdata, connection ):
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


def write_main_page( svg_name, out_path ):
    content = f"""\
<html>
    <head>
        <style>
            body {{ padding: 24;
                    background-color: #bbbbbb;
                 }}
            pre {{ background-color: rgb(226, 226, 226);
                   margin: 0px;
                   margin-top: 24px;
                   padding: 16px;
                }}
            pre code {{ margin: 0px;
                        padding: 0px;
                     }}
        
            .center_content {{ width: 100%;
                               margin-right: auto; margin-left: auto;
                               text-align: center;
                               padding-top: 24; padding-bottom: 24;
                            }}
            .info_content {{ margin-bottom: 36;
                          }}
        
        </style>
    </head>

    <body>
        <div class="top_content">Main graph</div>

        <div class="center_content">
            <object type="image/svg+xml" data="{svg_name}">missing image</object>
        </div>
    </body>

</html>
"""
    write_file( out_path, content )


def write_message_page( item, out_path ):
    msg_data = data_to_dict( item.msgdata )
    msg_data = pprint.pformat( msg_data, indent=1, width=1, sort_dicts=False )

    content = f"""\
<html>
    <head>
        <style>
            body {{ padding: 24;
                    background-color: #bbbbbb;
                 }}
            pre {{ background-color: rgb(226, 226, 226);
                   margin: 0px;
                   margin-top: 24px;
                   padding: 16px;
                }}
            pre code {{ margin: 0px;
                        padding: 0px;
                     }}
        
            .center_content {{ width: 100%;
                               margin-right: auto; margin-left: auto;
                               text-align: center;
                               padding-top: 24; padding-bottom: 24;
                            }}
            .info_content {{ margin-bottom: 36;
                          }}
        
        </style>
    </head>

    <body>
        <div class="top_content">
            <a href="../full_graph.html">back to Main graph</a>
            <br />
        </div>
        <div class="info_content">
            Message: <code>{item.msgtype}</code><br/>
            <pre><code>{item.msgdef}</code></pre>
        </div>
        <div class="info_content">
            Data:
            <pre><code>{msg_data}</code></pre>
        </div>
    </body>

</html>
"""
    write_file( out_path, content )


def data_to_dict( data_obj ):
    if isinstance( data_obj, list ):
        return [ data_to_dict( item ) for item in data_obj ]
    try:
        data_dict = data_obj.__dict__
        for key, val in data_dict.copy().items():
            data_dict[ key ] = data_to_dict( val )
        type_key = "__msgtype__"
        if type_key not in data_dict.keys():
            return data_dict

        ## move '__msgtype__' key to top
        reordered_dict = {}
        reordered_dict[ type_key ] = data_dict[ type_key ]
        del data_dict[ type_key ]
        for key, val in data_dict.items():
            reordered_dict[ key ] = data_to_dict( val )
        return reordered_dict

    except AttributeError: # as ex:
        return data_obj

#     return json.dumps( data_obj )
    
#     except Exception as ex:
#         print( ex )
#         return data_obj


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
    parser.add_argument( '--write_messages', action='store_true', help="Write messages" )
    parser.add_argument( '--no-detect_loops', action='store_false', help="Detect message loops and group in one UML loop" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    generate( args.bag_path, args.topic_dump_dir, args.outdir, args.group_topics, args.group_subs, args.no_detect_loops, args.write_messages )


if __name__ == '__main__':
    import argparse

    main()
