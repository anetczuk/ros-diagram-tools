# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import pprint
import copy
import re
import collections

from typing import List, Dict, Any, Set

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
from rosdiagram.seqgraph import DiagramData, NodeData, TopicData, MsgData
from rosdiagram.plantumltohtml import generate_plantuml_html, data_to_dict

from rosdiagram.seqgraph import NotesContainer      ## import interface
from rosdiagram.plantuml import format_note_error   ## import interface


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def get_msg_value_name( data, attribute_name, enum_prefix="" ):
    attr_val, enum_name = get_msg_value_enum( data, attribute_name, enum_prefix )
    if enum_name is None:
        message = format_note_error( "unknown enum" )
        return f"""'{attr_val}' ({message})"""
    return f"'{attr_val}' ({enum_name})"


def get_msg_name_enum( data, attribute_name, enum_prefix="" ):
    value_name = get_msg_value_enum( data, attribute_name, enum_prefix )
    return value_name[1]


def get_msg_value_enum( data, attribute_name, enum_prefix="" ):
    value = getattr( data, attribute_name )
    name  = None
    data_items = dir( data )
    for attr in data_items:
        if attr == attribute_name:
            continue
        if attr.startswith( enum_prefix ) is False:
            continue
        attr_val = getattr(data, attr)
        if attr_val == value:
            name = attr
    return (value, name)


## ===================================================================


##
def generate( bag_path, topic_dump_dir, outdir, exclude_set=None, params: dict = None ):
    if params is None:
        params = {}

    exclude_filter = ExcludeItemFilter( exclude_set )

    _LOGGER.info( "exclude set: %s", exclude_filter.raw_exclude )

    topic_data = read_topics( topic_dump_dir )
    if topic_data is None:
        topic_data = {}

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

            nodes_subdir    = "nodes"
            topics_subdir   = "topics"
            msgs_subdir     = "msgs"
            msgtypes_subdir = "msgtypes"

            os.makedirs( os.path.join( outdir, nodes_subdir ), exist_ok=True )
            os.makedirs( os.path.join( outdir, topics_subdir ), exist_ok=True )
            os.makedirs( os.path.join( outdir, msgs_subdir ), exist_ok=True )
            os.makedirs( os.path.join( outdir, msgtypes_subdir ), exist_ok=True )

            diagram_data: DiagramData = calculate_diagram_data( reader, params, topic_data, exclude_filter,
                                                                nodes_subdir, topics_subdir )

            seq_diagram = diagram_data.seq_diagram
            items_count = seq_diagram.itemsNum()
            _LOGGER.info( "diagram items num: %s", items_count )

            ## calculate notes
            _LOGGER.info( "calculating notes" )
            notes_functor = params.get( 'notes_functor' )
            if notes_functor is not None:
                for loop in seq_diagram.getLoops():
                    ## item: MsgData
                    for item in loop.items:
                        if item.isMessageSet() is False:
                            continue
                        note_content = notes_functor( item.topics, item.msgtype, item.msgdata )
                        item.notes_data = note_content

            ## generating message data
            _LOGGER.info( "generating messages data" )
            diagram_data.root_subdir = "../"
            message_pages_list = generate_messages_list( diagram_data, msgs_subdir, outdir )

            msgtypes_dict = generate_message_types_dict( diagram_data, msgtypes_subdir, outdir )

            ## generating main data
            _LOGGER.info( "generating main data" )
            diagram_data.root_subdir = ""
            main_page_dict = generate_main_dict( diagram_data, bag_path, exclude_set, outdir )

            ## generating nodes pages
            _LOGGER.info( "generating nodes data" )
            diagram_data.root_subdir = "../"
            node_pages_list = generate_nodes_list( diagram_data, outdir )

            ## generating topic data
            _LOGGER.info( "generating topics data" )
            diagram_data.root_subdir = "../"
            topic_pages_list = generate_topics_list( diagram_data, outdir )

            params_dict = { "style": {},
                            "main_page": main_page_dict,
                            "node_pages": node_pages_list,
                            "topic_pages": topic_pages_list,
                            "message_pages": message_pages_list,
                            "msgtypes_dict": msgtypes_dict
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


## 'topic_subs' -- dictionary containing topics subscribers
def calculate_diagram_data( reader, params, topic_data, exclude_filter, nodes_subdir, topics_subdir ) -> DiagramData:
    topic_subs = get_topic_subs_dict( topic_data )

    excluded_nodes = get_excluded_nodes( topic_subs, exclude_filter )

    excluded_topics = set()

    _LOGGER.info( "iterating rosbag connections: %s", len(reader.connections) )

    ## topics list
    topics_dict: Dict[str, Any] = {}                   ## it happened that topic was split into more than one connection

    ## connection: rosbags.interfaces.Connection
    for connection in reader.connections:
        curr_topic = connection.topic

        topic_obj = topics_dict.get( curr_topic, None )
        if topic_obj is None:
            topic_obj = TopicData( curr_topic, connection.msgcount )

            cuur_topic_data = topic_data.get( curr_topic, {} )
            topic_obj.pubs = cuur_topic_data.get( "pubs", [] )
            topic_obj.subs = cuur_topic_data.get( "subs", [] )

            topic_obj.msgtype = connection.msgtype

            is_excluded = exclude_filter.excluded( curr_topic )
            if is_excluded:
                excluded_topics.add( curr_topic )
            else:
                topic_filename = prepare_filesystem_name( curr_topic )
                topic_obj.suburl = os.path.join( topics_subdir, topic_filename + ".html" )

            topic_obj.excluded = is_excluded
            topics_dict[ curr_topic ] = topic_obj
        else:
            ## next connection with the same topic found
            topic_obj.msgcount += connection.msgcount

    topics_data: List[ TopicData ] = list( topics_dict.values() )

    ## generating sequence diagram
    _LOGGER.info( "generating sequence graph" )
    seq_diagram: SequenceGraph = generate_basic_graph( reader, topic_subs, excluded_topics )
    seq_diagram.process( params )

    ## nodes list
    nodes_data: List[ NodeData ] = []
    graph_actors: Set[str] = seq_diagram.actors()
    for node_name in graph_actors:
        node_filename     = prepare_filesystem_name( node_name )
        node_obj          = NodeData( node_name )
        node_obj.suburl   = os.path.join( nodes_subdir, node_filename + ".html" )
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

    ret_data.sortNodes()
    ret_data.sortTopics()

    return ret_data


def generate_main_dict( diagram_data: DiagramData, bag_path, exclude_set, outdir ):
    seq_diagram: SequenceGraph = diagram_data.seq_diagram

    bag_name = os.path.basename( bag_path )

    out_path = os.path.join( outdir, f"flow_{bag_name}.puml" )
    generate_diagram( diagram_data, out_path  )

    nodes_data: List[ NodeData ]   = diagram_data.nodes
    topics_data: List[ TopicData ] = diagram_data.topics

    errors_data: List[ Any ] = []
    messages: List[MsgData] = seq_diagram.messages()
    for msg_data in messages:
        notes_data: NotesContainer = msg_data.notes_data
        if notes_data is None:
            continue
        error_notes = notes_data.getErrorNotes()
        if len(error_notes) < 1:
            continue
        timestamp_dt     = msg_data.getTimestampDateTime()
        timestamp_string = timestamp_dt.strftime('%H:%M:%S.%f')

        url_list = diagram_data.getTopicsUrls( msg_data.topics )

        errors_data.append( { 'msg': msg_data,
                              'url': msg_data.getProp('url'),
                              'timestamp': timestamp_string,
                              'topics': url_list,
                              'notes': error_notes
                              } )

    svg_path = f"flow_{bag_name}.svg"

    page_dict = { 'bag_file': bag_path,
                  'svg_name': svg_path,
                  'nodes_data':  nodes_data,
                  'topics_data': topics_data,
                  'errors_data': errors_data,
                  'exclude_set': exclude_set
                  }
    return page_dict


def generate_nodes_list( diagram_data: DiagramData, outdir ):
    ret_params_list = []

    seq_diagram   = diagram_data.seq_diagram
    nodes_data    = diagram_data.nodes
    params        = diagram_data.params

    ## node_data: List[ NodeData ]
    for node_data in nodes_data:
        if node_data.excluded:
            continue

        actor = node_data.name

        _LOGGER.info( "preparing sequence graph for node %s", actor )

        sub_diagram: SequenceGraph = seq_diagram.copyCallingsActors( actor )
        sub_diagram.process( params )

        subdiagram_data: DiagramData  = copy.copy( diagram_data )        ## shallow copy
        subdiagram_data.seq_diagram   = sub_diagram
        subdiagram_data.nodes         = copy.deepcopy( diagram_data.nodes )        ## deep copy

        sugdiagram_node = subdiagram_data.getNodeByName( actor )
        if sugdiagram_node:
            sugdiagram_node.params[ "bg_color" ] = "LawnGreen"
            ## sugdiagram_node.params[ "bg_color" ] = "LimeGreen"

        diag_nodes  = sub_diagram.getActors()
        diag_topics = sub_diagram.getTopics()

        subdiagram_data.root_subdir = "../"

        subdiagram_data.nodes  = subdiagram_data.filterNodes( diag_nodes )
        subdiagram_data.topics = subdiagram_data.filterTopics( diag_topics )

        subdiagram_data.sortNodes()
        subdiagram_data.sortTopics()

        out_path     = os.path.join( outdir, node_data.suburl )
        path_name, _ = os.path.splitext( out_path )

        puml_out_path = f"{path_name}.puml"
        _LOGGER.info( "preparing puml diagram %s", out_path )
        generate_diagram( subdiagram_data, puml_out_path )

        node_filename = prepare_filesystem_name( node_data.name )
        svg_path      = node_filename + ".svg"

        page_dict = { 'out_path': out_path,
                      'node_info': node_data,
                      'svg_name': svg_path,
                      'root_url': subdiagram_data.root_subdir,
                      'nodes_data':  subdiagram_data.nodes,
                      'topics_data': subdiagram_data.topics
                      }
        ret_params_list.append( page_dict )

    return ret_params_list


def generate_topics_list( diagram_data: DiagramData, outdir ):
    ret_params_list = []

    seq_diagram   = diagram_data.seq_diagram
    topics_data   = diagram_data.topics
    params        = diagram_data.params

    ## topic_data: TopicData
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

        diag_nodes  = sub_diagram.getActors()
        diag_topics = sub_diagram.getTopics()

        subdiagram_data.nodes  = subdiagram_data.filterNodes( diag_nodes )
        subdiagram_data.topics = subdiagram_data.filterTopics( diag_topics )

        subdiagram_data.sortNodes()
        subdiagram_data.sortTopics()

        subdiagram_data.root_subdir = "../"

        out_path     = os.path.join( outdir, topic_data.suburl )
        path_name, _ = os.path.splitext( out_path )

        puml_out_path = f"{path_name}.puml"
        _LOGGER.info( "preparing puml diagram %s", out_path )
        generate_diagram( subdiagram_data, puml_out_path )

        topic_filename = prepare_filesystem_name( topic_data.name )
        svg_path       = topic_filename + ".svg"

        pubs_url_list = diagram_data.getNodesUrls( topic_data.pubs )
        subs_url_list = diagram_data.getNodesUrls( topic_data.subs )

        page_dict = { 'out_path': out_path,
                      'topic_info': topic_data,
                      'svg_name': svg_path,
                      'root_url': subdiagram_data.root_subdir,
                      'nodes_data':  subdiagram_data.nodes,
                      'topics_data': subdiagram_data.topics,
                      'pub_urls': pubs_url_list,
                      'sub_urls': subs_url_list
                      }
        ret_params_list.append( page_dict )

    return ret_params_list


def generate_messages_list( diagram_data: DiagramData, msgs_subdir, outdir ):
    seq_diagram: SequenceGraph = diagram_data.seq_diagram
    params = diagram_data.params

    if params.get( "write_messages", False ) is False:
        return []

    ret_params_list = []

    messages: List[MsgData] = seq_diagram.messages()
    for item in messages:
        out_url = os.path.join( msgs_subdir, f"{item.index:07d}_msg.html" )
        item.setProp( "url", out_url )
        out_path = os.path.join( outdir, out_url )
        notes_content = item.notes_data

        timestamp_dt = item.getTimestampDateTime()

        time_value, time_unit = convert_time_index( item.timestamp_rel )

        msg_data = data_to_dict( item.msgdata )
        msg_data = pprint.pformat( msg_data, indent=1, width=1, sort_dicts=False )              # type: ignore

        pub_url_list   = diagram_data.getNodesUrls( [ item.pub ] )
        subs_url_list  = diagram_data.getNodesUrls( item.subs )
        topic_url_list = diagram_data.getTopicsUrls( item.topics )

        page_dict = { 'out_path': out_path,
                      'timestamp': timestamp_dt,
                      'time_value': time_value,
                      'time_unit': time_unit,
                      'item': item,
                      'pub_url': pub_url_list[0],
                      'sub_urls': subs_url_list,
                      'topic_urls': topic_url_list,
                      'msg_data': msg_data,
                      'notes_content': notes_content
                      }
        ret_params_list.append( page_dict )

    return ret_params_list


def generate_message_types_dict( diagram_data: DiagramData, msgtypes_subdir, outdir ):
    seq_diagram: SequenceGraph = diagram_data.seq_diagram
    params = diagram_data.params

    if params.get( "write_messages", False ) is False:
        return {}

    ret_params_dict: Dict[ str, Any ] = {}

    ## loop: List[ SeqItems ]
    for loop in seq_diagram.getLoops():
        if loop.repeats > 1:
            pass
        ## item: List[ MsgData ]
        for item in loop.items:
            if item.isMessageSet() is False:
                continue

            msgtype = item.msgtype
            type_obj = ret_params_dict.get( msgtype, None )
            if type_obj:
                continue

            msgtype_filename = prepare_filesystem_name( msgtype )
            out_url  = os.path.join( msgtypes_subdir, f"{msgtype_filename}.html" )
            out_path = os.path.join( outdir, out_url )

            topic_url_list = diagram_data.getTopicsUrls( item.topics )

            page_dict = { 'out_path':   out_path,
                          'suburl':     out_url,
                          'msgtype':    msgtype,
                          'topic_urls': topic_url_list,
                          'msgdef':     item.msgdef
                          }
            ret_params_dict[ msgtype ] = page_dict

    return ret_params_dict


## 'topic_subs' -- dictionary containing topics subscribers
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
        graph_item = seq_diagram.addCallSubs( msg_index, topic_publisher, subscribers,
                                              time_diff, timestamp, connection.topic )

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
