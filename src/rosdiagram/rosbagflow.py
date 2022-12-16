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
import pprint
import datetime
import copy

import numpy

import argparse


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


if __name__ == '__main__':
    ## allow having executable script inside package and have proper imports
    ## replace directory of main package (prevent inconsistent imports)
    sys.path[0] = os.path.join( SCRIPT_DIR, os.pardir )


import rosbags
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_ros1
from rosbags.typesys import get_types_from_msg, register_types

from rosdiagram.io import read_list, prepare_filesystem_name
from rosdiagram.rostopictree import read_topics, get_topic_subs_dict
from rosdiagram.plantuml import SequenceGraph, generate_seq_diagram,\
    convert_time_index
from rosdiagram.seqgraph import GraphItem
from rosdiagram import texttemplate


## ===================================================================


def get_msg_value_name( data, attribute, enum_prefix="" ):
    attr_val  = getattr( data, attribute )
    enum_name = get_msg_name_enum( data, attribute, enum_prefix )
    if enum_name is None:
        message = format_note_error( "???" )
        return f"""'{attr_val}' ({message})"""
    return f"'{attr_val}' ({enum_name})"


def get_msg_name_enum( data, attribute, enum_prefix ):
    value = getattr( data, attribute )
    name = None
    data_items = dir( data )
    for attr in data_items:
        if attr == attribute:
            continue
        if attr.startswith( enum_prefix ) is False:
            continue
        attr_val = getattr(data, attr)
        if attr_val == value:
            name = attr
    return name


def format_note_error( message: str ):
    return f"""<b><back:salmon>{message}</back></b>"""
    

## ===================================================================


def generate( bag_path, topic_dump_dir, outdir, exclude_set=None, params: dict = None, notes_functor=None ):
    if exclude_set is None:
        exclude_set = set()
    if params is None:
        params = {}

    print( "exclude set:", exclude_set )

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

    try:
        # create reader instance and open for reading
        with Reader( bag_path ) as reader:
            print( "bag statistics:" )
            print( "total messages:", reader.message_count )
            print( "time span:", str( reader.duration / 1000000000 / 60 ) + "m" )
    
            # topic and msgtype information is available on .connections list
    
            ## generating sequence diagram
            seq_diagram: SequenceGraph = generate_basic_graph( reader, topic_subs, exclude_set )
            seq_diagram.process( params )
            
            items_count = seq_diagram.itemsNum()
            print( "diagram items num:", items_count )
    
            ## generating actors pages
            nodes_subdir = "nodes"
            nodes_out_dir = os.path.join( outdir, nodes_subdir )
            os.makedirs( nodes_out_dir, exist_ok=True )
            graph_actors = seq_diagram.actors()
            for actor in graph_actors:
                actor_filename = prepare_filesystem_name( actor )
                sub_diagram: SequenceGraph = seq_diagram.copyCallings( actor )
                sub_diagram.process( params )
    
                if params.get( "write_messages", False ):
                    out_dir = os.path.join( outdir, "msgs" )
                    os.makedirs( out_dir, exist_ok=True )
                    loops = sub_diagram.getLoops()
                    for loop in loops:
                        if loop.repeats > 1:
                            pass
                        for item in loop.items:
                            if item.isMessageSet() is False:
                                continue
                            out_name = f"{item.index}_msg.html"
                            item.setProp( "url", "../msgs/" + out_name )
    
                out_path = os.path.join( nodes_out_dir, f"{actor_filename}.puml" )
                generate_seq_diagram( sub_diagram, out_path, params, nodes_subdir="", notes_functor=notes_functor )
    
                svg_path = actor_filename + ".svg"
                out_path = os.path.join( nodes_out_dir, actor_filename + ".html" )
                write_seq_page( None, svg_path, "", out_path )

            ## generating message pages
            if params.get( "write_messages", False ):
                out_dir = os.path.join( outdir, "msgs" )
                os.makedirs( out_dir, exist_ok=True )
                loops: List[ SeqItems ] = seq_diagram.getLoops()
                for loop in loops:
                    if loop.repeats > 1:
                        pass
                    for item in loop.items:
                        if item.isMessageSet() is False:
                            continue
                        out_name = f"{item.index}_msg.html"
                        item.setProp( "url", "msgs/" + out_name )
                        out_path = os.path.join( out_dir, out_name )
                        write_message_page( item, out_path )
    
            ## write main page
            topics_data = []
            for connection in reader.connections:
                topics_data.append( ( connection.topic, connection.msgcount ) )
            topics_data = sorted( topics_data, key=lambda x: (-x[1], x[0]) )
    
            topics_content = "Topics list:<br/>"
            topics_content += "<ul>\n"
            for topic_item in topics_data:
                topics_content += f"<li><code>{topic_item[0]}</code>: {topic_item[1]}</li>\n"
            topics_content += "</ul>\n"
    
            ## write main diagram
            bag_name = os.path.basename( bag_path )
    
            out_path = os.path.join( outdir, f"flow_{bag_name}.puml" )
            generate_seq_diagram( seq_diagram, out_path, params, nodes_subdir=nodes_subdir, notes_functor=notes_functor )
    
            svg_path = f"flow_{bag_name}.svg"
            main_out_path = os.path.join( outdir, "full_graph.html" )
            write_seq_page( bag_path, svg_path, topics_content, main_out_path )
    
            print( f"generated main page: file://{main_out_path}" )
    except rosbags.rosbag1.reader.ReaderError as ex:
        _LOGGER.error( "unable to parse bag file: %s", ex )


def generate_basic_graph( reader, topic_subs, exclude_set ):
    # iterate over messages
    ## iterates items in timestamp order
    messages = reader.messages()
    if not messages:
        print( "no message found" )
        return None

    seq_diagram = SequenceGraph()
    first_item = next( messages )
    first_timestamp = first_item[1]

    messages = reader.messages()
    for connection, timestamp, rawdata in messages:
        if connection.topic in exclude_set:
            continue

        subscribers = topic_subs[ connection.topic ]
        if not subscribers:
            ## topic without subscribers
            continue

        ext = connection.ext
        topic_publisher = ext.callerid

        time_diff = timestamp - first_timestamp
        graph_item = seq_diagram.addCallSubs( topic_publisher, subscribers, time_diff, timestamp, connection.topic )

        valid, msg = deserialize_msg( rawdata, connection )
        if valid is False:
            print( "unable to deserialize:", timestamp, connection.topic, connection.msgtype )
            continue
        graph_item.setMessageData( connection.msgtype, connection.msgdef, msg )

    return seq_diagram


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


def write_seq_page( bag_file, svg_name, bottom_content, out_path ):
    print( f"generating page: file://{out_path}" )

    template_path = os.path.join( SCRIPT_DIR, "template", "baggraph_seq_page.html.tmpl" )

    page_params = { 'bag_file': bag_file,
                    'svg_name': svg_name,
                    'bottom_content': bottom_content
                    }
    texttemplate.generate( template_path, out_path, INPUT_DICT=page_params )


def write_message_page( item: GraphItem, out_path ):
    timestamp_dt = datetime.datetime.fromtimestamp( item.timestamp / 1000000000 )

    time_value, time_unit = convert_time_index( item.index )

    msg_data = data_to_dict( item.msgdata )
    msg_data = pprint.pformat( msg_data, indent=1, width=1, sort_dicts=False )              # type: ignore

    template_path = os.path.join( SCRIPT_DIR, "template", "baggraph_message.html.tmpl" )

    page_params = { 'timestamp_dt': timestamp_dt,
                    'time_value': time_value,
                    'time_unit': time_unit,
                    'item': item,
                    'msg_data': msg_data
                    }
    texttemplate.generate( template_path, out_path, INPUT_DICT=page_params )


def data_to_dict( data_obj ):
    if isinstance( data_obj, list ):
        return [ data_to_dict( item ) for item in data_obj ]

    if isinstance( data_obj, numpy.ndarray ):
        return list( data_obj )

    try:
        data_dict = None
        if isinstance( data_obj, dict ):
            data_dict = copy.copy( data_obj )
        else:
            data_dict = copy.copy( data_obj.__dict__ )
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

    # except AttributeError as ex:
    except AttributeError:
        ## object has no attribute '__dict__'
        return data_obj


## ===================================================================


def main( notes_functor=None ):
    parser = argparse.ArgumentParser(description='catkin deps tree')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--bag_path', action='store', required=True, default="",
                         help="Path to rosbag file" )
    parser.add_argument( '--topic_dump_dir', action='store', required=False, default="",
                         help="Dump directory containing 'rostopic' output data" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )
    parser.add_argument( '--group_calls', action='store_true', help="Group calls to same topic" )
    parser.add_argument( '--group_topics', action='store_true', help="Group multiple topics in one call" )
    parser.add_argument( '--group_subs', action='store_true', help="Group topic's subscribers in one UML group" )
    parser.add_argument( '--detect_loops', action='store_true', help="Detect message loops and group in one UML loop" )
    parser.add_argument( '--write_messages', action='store_true', help="Write message subpages" )
    parser.add_argument( '--exclude_list_path', action='store', help="Topics exclude list path" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    try:
        exclude_list = read_list( args.exclude_list_path )
        exclude_list = set( exclude_list )
    except TypeError as ex:
        _LOGGER.warning( "unable to load exception list: %s", ex )
        exclude_list = set()

    params = { "group_calls": args.group_calls,
               "group_topics": args.group_topics,
               "group_subs": args.group_subs,
               "detect_loops": args.detect_loops,
               "write_messages": args.write_messages
               }
    generate( args.bag_path, args.topic_dump_dir, args.outdir, exclude_list, params, notes_functor )


if __name__ == '__main__':
    main()
