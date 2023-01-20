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
import collections

from typing import List

import argparse

import rosbags
from pympler import asizeof

from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_ros1
from rosbags.typesys import get_types_from_msg, register_types

from rosdiagram.io import read_list, prepare_filesystem_name
from rosdiagram.ros.rostopicdata import read_topics, get_topic_subs_dict
from rosdiagram.plantuml import SequenceGraph, generate_diagram,\
    convert_time_index
from rosdiagram.seqgraph import DiagramData, NodeData, TopicData
from rosdiagram.plantumltohtml import generate_plantuml_html, data_to_dict


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


##
def generate( bag_path, topic_dump_dir, outdir, exclude_set=None, params: dict = None ):
    if params is None:
        params = {}

    exclude_filter = ExcludeItemFilter( exclude_set )

    _LOGGER.info( "exclude set: %s", exclude_filter.raw_exclude )

    topic_data  = read_topics( topic_dump_dir )
    if topic_data is None:
        topic_data = {}
    topic_subs  = get_topic_subs_dict( topic_data )

#     _LOGGER.info( "got topic data: %s", topic_data )
#     _LOGGER.info( "got topic subs: %s", topic_subs )

    try:
        # create reader instance and open for reading
        with Reader( bag_path ) as reader:
            bag_time_span = reader.duration / 1000000000 / 60
            _LOGGER.info( """bag statistics:
total messages: %s
time span: %sm""", reader.message_count, bag_time_span )

            print_topics_stats( reader )

            # topic and msgtype information is available on .connections list

            diagram_data: DiagramData = calculate_diagram_data( reader, params, topic_subs, exclude_filter )

            nodes_subdir  = "nodes"
            topics_subdir = "topics"

            diagram_data.nodes_subdir  = nodes_subdir
            diagram_data.topics_subdir = topics_subdir
            diagram_data.msgs_subdir   = "msgs"

            seq_diagram = diagram_data.seq_diagram
            items_count = seq_diagram.itemsNum()
            _LOGGER.info( "diagram items num: %s", items_count )

            ## calculate notes
            _LOGGER.info( "calculating notes" )
            notes_functor = params.get( 'notes_functor' )
            if notes_functor is not None:
                for loop in seq_diagram.getLoops():
                    for item in loop.items:
                        if item.isMessageSet() is False:
                            continue
                        note_content = notes_functor( item.topics, item.msgtype, item.msgdata )
                        item.notes_data = note_content

            ## generating message data
            _LOGGER.info( "generating messages data" )
            message_pages_list = generate_messages_list( diagram_data, outdir )

            ## generating main data
            _LOGGER.info( "generating main data" )
            main_page_dict = generate_main_dict( diagram_data, bag_path, exclude_set, outdir )

            ## generating nodes pages
            _LOGGER.info( "generating nodes data" )
            diagram_data.topics_subdir = topics_subdir
            node_pages_list = generate_nodes_list( diagram_data, outdir )

            ## generating topic data
            _LOGGER.info( "generating topics data" )
            topic_pages_list = generate_topics_list( diagram_data, outdir )

            params_dict = { "style": {},
                            "main_page": main_page_dict,
                            "node_pages": node_pages_list,
                            "topic_pages": topic_pages_list,
                            "message_pages": message_pages_list
                            }
            generate_plantuml_html( outdir, params_dict )

    except rosbags.rosbag1.reader.ReaderError as ex:
        _LOGGER.error( "unable to parse bag file: %s", ex )


def print_topics_stats( reader ):
    msg_num_counter = collections.Counter()
    for connection in reader.connections:
        curr_topic  = connection.topic
        msg_num     = connection.msgcount
        msg_num_counter[ curr_topic ] += msg_num

    msg_size_counter = collections.Counter()
    messages = reader.messages()
    for connection, _, rawdata in messages:
        curr_topic  = connection.topic
        curr_size = asizeof.asizeof( rawdata )
        msg_size_counter[ curr_topic ] += curr_size

    topics_data = []
    for topic, count in msg_num_counter.most_common():
        transfer_size = msg_size_counter.get( topic, 0 )
        transfer_label = ""
        if transfer_size < 1024:
            transfer_label = f"{transfer_size} B"
        elif transfer_size < 1024 * 1024:
            value = transfer_size / 1024
            value = round( value, 2 )
            transfer_label = f"{value} KB"
        else:
            value = transfer_size / (1024 * 1024)
            value = round( value, 2 )
            transfer_label = f"{value} MB"
        topics_data.append( ( topic, count, transfer_size, transfer_label ) )

    content = ""
    topics_data.sort( key=lambda item: item[1], reverse=True )
    for topic_item in topics_data:
        content += f"{topic_item[0]} {topic_item[1]} total size: {topic_item[3]}\n"
    _LOGGER.info( "topic count stats:\n%s", content )

    content = ""
    topics_data.sort( key=lambda item: item[2], reverse=True )
    for topic_item in topics_data:
        content += f"{topic_item[0]} {topic_item[1]} total size: {topic_item[3]}\n"
    _LOGGER.info( "topic memory stats:\n%s", content )


def calculate_diagram_data( reader, params, topic_subs, exclude_filter ) -> DiagramData:
    excluded_nodes = get_excluded_nodes( topic_subs, exclude_filter )

    excluded_topics = set()

    _LOGGER.info( "iterating rosbag connections: %s", len(reader.connections) )

    ## topics list
    topics_data: List[ TopicData ] = []
    ## connection: rosbags.interfaces.Connection
    for connection in reader.connections:
        curr_topic = connection.topic
        topic_obj  = TopicData( curr_topic, connection.msgcount )
        topic_obj.msgtype = connection.msgtype

        is_excluded = exclude_filter.excluded( curr_topic )
        if is_excluded:
            excluded_topics.add( curr_topic )
        else:
            topic_filename = prepare_filesystem_name( curr_topic )
            topic_obj.suburl = topic_filename + ".html"

        topic_obj.excluded = is_excluded
        topics_data.append( topic_obj )
    topics_data = sorted( topics_data, key=lambda x: (-x[1], x[0]) )

    ## generating sequence diagram
    _LOGGER.info( "generating sequence graph" )
    seq_diagram: SequenceGraph = generate_basic_graph( reader, topic_subs, excluded_topics )
    seq_diagram.process( params )

    ## nodes list
    nodes_data: List[ NodeData ] = []
    graph_actors = seq_diagram.actors()
    for node in graph_actors:
        node_filename = prepare_filesystem_name( node )
        node_obj          = NodeData( node )
        node_obj.suburl   = node_filename + ".html"
        node_obj.excluded = False
        nodes_data.append( node_obj )
    for node in excluded_nodes:
        node_obj          = NodeData( node )
        node_obj.excluded = True
        nodes_data.append( node_obj )

    ret_data = DiagramData()
    ret_data.seq_diagram = seq_diagram
    ret_data.nodes       = nodes_data
    ret_data.topics      = topics_data
    ret_data.params      = params

    return ret_data


def generate_main_dict( diagram_data: DiagramData, bag_path, exclude_set, outdir ):
    bag_name = os.path.basename( bag_path )

    out_path = os.path.join( outdir, f"flow_{bag_name}.puml" )
    generate_diagram( diagram_data, out_path  )

    nodes_data: List[ NodeData ]   = diagram_data.nodes
    topics_data: List[ TopicData ] = diagram_data.topics

    for node in nodes_data:
        if node.suburl is not None:
            node.suburl = os.path.join( diagram_data.nodes_subdir, node.suburl )
    ## topic: TopicData
    for topic in topics_data:
        if topic.suburl is not None:
            topic.suburl = os.path.join( diagram_data.topics_subdir, topic.suburl )

    svg_path = f"flow_{bag_name}.svg"

    page_dict = { 'bag_file': bag_path,
                  'svg_name': svg_path,
                  'nodes_data':  nodes_data,
                  'topics_data': topics_data,
                  'exclude_set': exclude_set
                  }
    return page_dict


def generate_nodes_list( diagram_data: DiagramData, outdir ):
    ret_params_list = []

    seq_diagram   = diagram_data.seq_diagram
    nodes_data    = diagram_data.nodes
    params        = diagram_data.params
    nodes_subdir  = diagram_data.nodes_subdir

    nodes_out_dir = os.path.join( outdir, nodes_subdir )
    os.makedirs( nodes_out_dir, exist_ok=True )

    ## node_data: List[ NodeData ]
    for node_data in nodes_data:
        if node_data.excluded:
            continue

        actor = node_data.name

        _LOGGER.info( "preparing sequence graph for node %s", actor )

        actor_filename = prepare_filesystem_name( actor )
        sub_diagram: SequenceGraph = seq_diagram.copyCallingsActors( actor )
        sub_diagram.process( params )

        subdiagram_data: DiagramData  = copy.copy( diagram_data )        ## shallow copy
        subdiagram_data.seq_diagram   = sub_diagram
        subdiagram_data.nodes_subdir  = ""
        subdiagram_data.topics_subdir = "../topics"
        subdiagram_data.msgs_subdir   = os.path.join( os.pardir, subdiagram_data.msgs_subdir )
        subdiagram_data.nodes         = copy.deepcopy( diagram_data.nodes )        ## eep copy

        sugdiagram_node = subdiagram_data.getNodeByName( actor )
        if sugdiagram_node:
            sugdiagram_node.params[ "bg_color" ] = "LimeGreen"

        out_path = os.path.join( nodes_out_dir, f"{actor_filename}.puml" )
        _LOGGER.info( "preparing puml diagram %s", out_path )
        generate_diagram( subdiagram_data, out_path )

        svg_path    = actor_filename + ".svg"
        actors_page = actor_filename + ".html"
        out_path    = os.path.join( nodes_out_dir, actors_page )

        page_dict = { 'out_path': out_path,
                      'svg_name': svg_path
                      }
        ret_params_list.append( page_dict )

    return ret_params_list


def generate_topics_list( diagram_data: DiagramData, outdir ):
    ret_params_list = []

    seq_diagram   = diagram_data.seq_diagram
    topics_data   = diagram_data.topics
    params        = diagram_data.params
    topics_subdir = diagram_data.topics_subdir

    topics_out_dir = os.path.join( outdir, topics_subdir )
    os.makedirs( topics_out_dir, exist_ok=True )

    for topic_data in topics_data:
        if topic_data.excluded:
            continue

        sub_diagram: SequenceGraph = seq_diagram.copyCallingsLabels( topic_data.name )
        if sub_diagram.size() < 1:
            topic_data.suburl = None
            continue

        sub_diagram.process( params )

        subdiagram_data: DiagramData = copy.copy( diagram_data )
        subdiagram_data.seq_diagram   = sub_diagram
        subdiagram_data.nodes_subdir  = os.path.join( os.pardir, subdiagram_data.nodes_subdir )
        subdiagram_data.topics_subdir = ""
        subdiagram_data.msgs_subdir   = os.path.join( os.pardir, subdiagram_data.msgs_subdir )

        topic_filename = prepare_filesystem_name( topic_data.name )
        out_path = os.path.join( topics_out_dir, f"{topic_filename}.puml" )
        generate_diagram( subdiagram_data, out_path )

        svg_path    = topic_filename + ".svg"
        actors_page = topic_filename + ".html"
        out_path    = os.path.join( topics_out_dir, actors_page )

        page_dict = { 'out_path': out_path,
                      'svg_name': svg_path
                      }
        ret_params_list.append( page_dict )

    return ret_params_list


def generate_messages_list( diagram_data: DiagramData, outdir ):
    seq_diagram: SequenceGraph = diagram_data.seq_diagram
    params = diagram_data.params

    if params.get( "write_messages", False ) is False:
        return []

    ret_params_list = []

    out_dir = os.path.join( outdir, "msgs" )
    os.makedirs( out_dir, exist_ok=True )

    ## loop: List[ SeqItems ]
    for loop in seq_diagram.getLoops():
        if loop.repeats > 1:
            pass
        ## item: List[ MsgData ]
        for item in loop.items:
            if item.isMessageSet() is False:
                continue
            out_name = f"{item.index:07d}_msg.html"
            item.setProp( "url", out_name )
            out_path = os.path.join( out_dir, out_name )
            note_content = item.notes_data
            if note_content is not None:
                note_content = note_content.replace( "\n", "<br />\n" )

            timestamp_dt = datetime.datetime.fromtimestamp( item.timestamp_abs / 1000000000 )

            time_value, time_unit = convert_time_index( item.timestamp_rel )

            msg_data = data_to_dict( item.msgdata )
            msg_data = pprint.pformat( msg_data, indent=1, width=1, sort_dicts=False )              # type: ignore

            page_dict = { 'out_path': out_path,
                          'timestamp_dt': timestamp_dt,
                          'time_value': time_value,
                          'time_unit': time_unit,
                          'item': item,
                          'msg_data': msg_data,
                          'notes_data': note_content
                          }
            ret_params_list.append( page_dict )

    return ret_params_list


def generate_basic_graph( reader, topic_subs, excluded_topics ):
    # iterate over messages
    ## iterates items in timestamp order
    messages = reader.messages()
    if not messages:
        _LOGGER.warning( "no message found" )
        return None

    seq_diagram = SequenceGraph()
    first_item = next( messages )
    first_timestamp = first_item[1]

    messages = reader.messages()
    msg_index = -1
    for connection, timestamp, rawdata in messages:
        msg_index += 1
        if connection.topic in excluded_topics:
            continue

        ext = connection.ext
        topic_publisher = ext.callerid

        subscribers = topic_subs.get( connection.topic, None )
        if not subscribers:
            ## topic without subscribers
            subscribers = ["void"]

        time_diff  = timestamp - first_timestamp
        graph_item = seq_diagram.addCallSubs( msg_index, topic_publisher, subscribers, time_diff, timestamp, connection.topic )

        valid, msg = deserialize_msg( rawdata, connection )
        if valid is False:
            _LOGGER.warning( "unable to deserialize: %s %s %s", timestamp, connection.topic, connection.msgtype )
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
