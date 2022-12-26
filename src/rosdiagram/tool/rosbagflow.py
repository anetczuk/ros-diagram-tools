# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import pprint
import datetime
import copy
import re

import argparse
import numpy

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


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def get_msg_value_name( data, attribute, enum_prefix="" ):
    attr_val  = getattr( data, attribute )
    enum_name = get_msg_name_enum( data, attribute, enum_prefix )
    if enum_name is None:
        message = format_note_error( "unknown enum" )
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


def generate( bag_path, topic_dump_dir, outdir, exclude_set=None, params: dict = None ):
    if params is None:
        params = {}

    exclude_filter = ExcludeItemFilter( exclude_set )

    print( "exclude set:", exclude_filter.raw_exclude )

    topic_data     = read_topics( topic_dump_dir )
    topic_subs     = get_topic_subs_dict( topic_data )
    excluded_nodes = get_excluded_nodes( topic_subs, exclude_filter )

    try:
        # create reader instance and open for reading
        with Reader( bag_path ) as reader:
            print( "bag statistics:" )
            print( "total messages:", reader.message_count )
            print( "time span:", str( reader.duration / 1000000000 / 60 ) + "m" )

            # topic and msgtype information is available on .connections list

            excluded_topics = set()

            topics_data = []
            for connection in reader.connections:
                curr_topic  = connection.topic
                is_excluded = exclude_filter.excluded( curr_topic )
                if is_excluded:
                    excluded_topics.add( curr_topic )
                topics_data.append( ( curr_topic, connection.msgcount, is_excluded ) )
            topics_data = sorted( topics_data, key=lambda x: (-x[1], x[0]) )

            ## generating sequence diagram
            seq_diagram: SequenceGraph = generate_basic_graph( reader, topic_subs, excluded_topics )
            seq_diagram.process( params )

            items_count = seq_diagram.itemsNum()
            print( "diagram items num:", items_count )

            ## generating actors pages
            nodes_data = []

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
                    for loop in sub_diagram.getLoops():
                        if loop.repeats > 1:
                            pass
                        for item in loop.items:
                            if item.isMessageSet() is False:
                                continue
                            out_name = f"{item.index}_msg.html"
                            item.setProp( "url", "../msgs/" + out_name )

                out_path = os.path.join( nodes_out_dir, f"{actor_filename}.puml" )
                generate_seq_diagram( sub_diagram, out_path, params, nodes_subdir="" )

                svg_path    = actor_filename + ".svg"
                actors_page = actor_filename + ".html"
                out_path    = os.path.join( nodes_out_dir, actors_page )

                nodes_data.append( (actor, os.path.join( nodes_subdir, actors_page ), False ) )

                write_seq_node_page( svg_path, out_path )

            for excluded in excluded_nodes:
                nodes_data.append( (excluded, None, True ) )

            ## generating message pages
            if params.get( "write_messages", False ):
                notes_functor = params.get( 'notes_functor' )
                out_dir = os.path.join( outdir, "msgs" )
                os.makedirs( out_dir, exist_ok=True )
                for loop in seq_diagram.getLoops():
                    if loop.repeats > 1:
                        pass
                    for item in loop.items:
                        if item.isMessageSet() is False:
                            continue
                        out_name = f"{item.index}_msg.html"
                        item.setProp( "url", "msgs/" + out_name )
                        out_path = os.path.join( out_dir, out_name )
                        note_content = None
                        if notes_functor is not None:
                            note_content = notes_functor( item.labels, item.msgtype, item.msgdata )
                        if note_content is not None:
                            note_content = note_content.replace( "\n", "<br />\n" )
                        write_message_page( item, out_path, note_content )

            ## write main page
            ## write main diagram
            bag_name = os.path.basename( bag_path )

            out_path = os.path.join( outdir, f"flow_{bag_name}.puml" )
            generate_seq_diagram( seq_diagram, out_path, params, nodes_subdir=nodes_subdir )

            svg_path = f"flow_{bag_name}.svg"
            main_out_path = os.path.join( outdir, "full_graph.html" )
            write_seq_main_page( bag_path, svg_path, nodes_data, topics_data, exclude_set, main_out_path )

            print( f"generated main page: file://{main_out_path}" )
    except rosbags.rosbag1.reader.ReaderError as ex:
        _LOGGER.error( "unable to parse bag file: %s", ex )


def generate_basic_graph( reader, topic_subs, excluded_topics ):
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
        if connection.topic in excluded_topics:
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


def get_excluded_nodes( topic_subs, exclude_filter ):
    excluded_nodes = set()
    for _, subs in topic_subs.items():
        for node in subs.copy():
            is_excluded = exclude_filter.excluded( node )
            if is_excluded is False:
                continue
            subs.remove( node  )
            excluded_nodes.add( node  )
    return excluded_nodes


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


##
class ExcludeItemFilter():

    def __init__(self, exclude_set=None):
        self.raw_exclude = exclude_set
        if self.raw_exclude is None:
            self.raw_exclude = set()
        for item in self.raw_exclude.copy():
            if len(item) < 1:
                self.raw_exclude.remove( item )

        self.exclude_set = set()
        self.regex_set   = set()

        for excl in self.raw_exclude:
            if "*" in excl:
                ## wildcard found
                pattern = excl
                pattern = pattern.replace( "*", ".*" )
                regex_obj = re.compile( pattern )
                self.regex_set.add( regex_obj )
            else:
                self.exclude_set.add( excl )

    def excluded(self, item):
        if item in self.exclude_set:
            return True
        for regex in self.regex_set:
            if regex.match( item ):
                return True
        return False


## ===================================================================


def write_seq_main_page( bag_file, svg_name, nodes_data, topics_data, exclude_set, out_path ):
    print( f"generating main page: file://{out_path}" )

    template_path = os.path.join( SCRIPT_DIR, "template", "baggraph_seq_main_page.html.tmpl" )

    page_params = { 'bag_file': bag_file,
                    'svg_name': svg_name,
                    'nodes_data': nodes_data,
                    'topics_data': topics_data,
                    'exclude_set': exclude_set
                    }
    texttemplate.generate( template_path, out_path, INPUT_DICT=page_params )


def write_seq_node_page( svg_name, out_path ):
    print( f"generating node page: file://{out_path}" )

    template_path = os.path.join( SCRIPT_DIR, "template", "baggraph_seq_node_page.html.tmpl" )

    page_params = { 'svg_name': svg_name
                    }
    texttemplate.generate( template_path, out_path, INPUT_DICT=page_params )


def write_message_page( item: GraphItem, out_path, notes_data=None ):
    timestamp_dt = datetime.datetime.fromtimestamp( item.timestamp / 1000000000 )

    time_value, time_unit = convert_time_index( item.index )

    msg_data = data_to_dict( item.msgdata )
    msg_data = pprint.pformat( msg_data, indent=1, width=1, sort_dicts=False )              # type: ignore

    template_path = os.path.join( SCRIPT_DIR, "template", "baggraph_message.html.tmpl" )

    page_params = { 'timestamp_dt': timestamp_dt,
                    'time_value': time_value,
                    'time_unit': time_unit,
                    'item': item,
                    'msg_data': msg_data,
                    'notes_data': notes_data
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
    parser = argparse.ArgumentParser(description='rosbag sequence diagram')
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
    parser.add_argument( '--exclude_list_path', action='store', help="Exclude list path" )

    args = parser.parse_args()

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    try:
        exclude_list = read_list( args.exclude_list_path )
    except TypeError as ex:
        _LOGGER.warning( "unable to load exception list: %s", ex )
        exclude_list = []

    params = { "group_calls": args.group_calls,
               "group_topics": args.group_topics,
               "group_subs": args.group_subs,
               "detect_loops": args.detect_loops,
               "write_messages": args.write_messages,
               "notes_functor": notes_functor
               }
    generate( args.bag_path, args.topic_dump_dir, args.outdir, exclude_list, params )
