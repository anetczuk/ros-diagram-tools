# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import datetime
import itertools
import logging
import hashlib

from typing import Set, List, Dict

from rosdiagram.io import write_file, prepare_filesystem_name, read_list
from rosdiagram.seqgraph import SequenceGraph, SeqItems, MsgData, DiagramData


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


BG_COLORS_PATH = os.path.join( SCRIPT_DIR, "plantuml_bg_colors.txt" )
BG_COLORS_LIST = read_list( BG_COLORS_PATH )


def generate_diagram( diagram_data: DiagramData, out_path ):
    params   = diagram_data.params
    genrator = SequenceDiagramGenerator( params,
                                         nodes_subdir=diagram_data.nodes_subdir,
                                         topics_subdir=diagram_data.topics_subdir,
                                         msgs_subdir=diagram_data.msgs_subdir )
    genrator.generate( diagram_data, out_path )


##
class SequenceDiagramGenerator():

    def __init__(self, params: dict = None, nodes_subdir="", topics_subdir="", msgs_subdir="" ):
        self.name_dict: Dict[str, str] = {}
        self.params_dict = params
        if self.params_dict is None:
            self.params_dict = {}

        self.nodes_subdir  = nodes_subdir
        self.topics_subdir = topics_subdir
        self.msgs_subdir   = msgs_subdir

        self.actors_order: List[str] = []

    def generate( self, diagram_data: DiagramData, out_path ):
        seq_graph    = diagram_data.seq_diagram

        call_len = seq_graph.size()
        if call_len < 1:
            content = """\
@startuml
@enduml
"""
            write_file( out_path, content )
            return

        content = """\
@startuml

skinparam backgroundColor #FEFEFE

"""

        graph_actors: Set[str] = seq_graph.actors()
        labels_dict            = self.calculateLabelsDict( seq_graph )
        self.actors_order      = calculate_actors_optimized_order( graph_actors, labels_dict )

        ## add actors
        for item in self.actors_order:
            item_id = self._getItemId( item )
#             content += f"""participant "{item}" as {item_id}\n"""

            item_filename = prepare_filesystem_name( item )
            item_path = item_filename + ".html"
            item_path = os.path.join( self.nodes_subdir, item_path )

            if BG_COLORS_LIST:
                ## BG_COLORS
                item_hash      = hashlib.sha256( item_id.encode('utf-8') ).hexdigest()
                bg_color_index = int( item_hash, 16 ) % len( BG_COLORS_LIST )
                bg_color       = BG_COLORS_LIST[ bg_color_index ]

                content += f"""box #{bg_color}\n"""
                ## content += f"""'bg color: {bg_color}\n"""
                content += f"""    participant "{item}" as {item_id} [[{item_path}]]\n"""
                content += "end box\n"
            else:
                content += f"""participant "{item}" as {item_id} [[{item_path}]]\n"""

        content += "\n"

        detect_loops = seq_graph.loopsFound()

        ## add calls
        loops = seq_graph.getLoops()
        for seq in loops:
            use_msg_loop = seq.repeats > 1 and detect_loops
            indent = ""

            if use_msg_loop:
                content += f"""\nloop {seq.repeats} times\n"""
                indent = "    "

            loop_content = self.generateLoop( seq, indent )

            content += loop_content
            if use_msg_loop:
                content += "end\n"

        content += "\n@enduml\n"

        write_file( out_path, content )

    def generateLoop( self, seq: SeqItems, loop_indent ):
        content    = ""
        group_subs = self.params_dict.get( "group_subs", False )

        calls: List[ MsgData ] = seq.items
        ## item: MsgData
        for msg_data in calls:
            receivers = sorted( msg_data.subs, reverse=True )

            message_url = None
            if msg_data.isMessageSet():
                message_url = msg_data.getProp( "url", None )
            if message_url is not None:
                message_url = os.path.join( self.msgs_subdir, message_url )

            ## topic url: out/topics/_turtle1_cmd_vel.html
            call_label = self.calculateLabel( msg_data, message_url )
            indent     = ""

            use_subs_group = len( receivers ) > 1 and group_subs
            if use_subs_group:
                ## grouping topic subscribers
                content += f"""{loop_indent}group {call_label}\n"""
                call_label = ""
                indent = "    "

            pub_id = self._getItemId( msg_data.pub )
            for rec in receivers:
                rec_id = self._getItemId( rec )
                content   += f"""{loop_indent}{indent}{pub_id} o-> {rec_id} : {call_label}\n"""
                if call_label:
                    if msg_data.notes_data is not None:
                        content += f"""\
note left
{msg_data.notes_data}
end note
"""
                call_label = ""     ## clear label after first item

            if use_subs_group:
                content += f"""{loop_indent}end\n"""

        return content

    def callTime(self, item: MsgData):
        timestamp_dt     = datetime.datetime.fromtimestamp( item.timestamp / 1000000000 )
        timestamp_string = timestamp_dt.strftime('%H:%M:%S.%f')
        return timestamp_string

    def calculateLabel(self, item: MsgData, url=None ):
        ret_content = ""

        timestamp_string = self.callTime( item )
        if url is None:
            ret_content = f"""**{timestamp_string}**: """
        else:
            ## {message data} is tooltip of hyperlink
            plantuml_url = generate_url( url, timestamp_string, "message data" )
            ret_content  = f"""**{plantuml_url}**: """

        labels_list = []
        for topic in item.topics:
            if self.topics_subdir:
                topic_filename = prepare_filesystem_name( topic ) + ".html"
                topic_path = os.path.join( self.topics_subdir, topic_filename )
                plantuml_url = generate_url( topic_path, topic, "topic data" )
                labels_list.append( plantuml_url )
            else:
                labels_list.append( topic )

        ret_content += " | ".join( labels_list )
        return ret_content

    def calculateLabelsDict(self, seq_graph: SequenceGraph ):
        labels_dict = {}
        loops: List[ SeqItems ] = seq_graph.getLoops()
        for seq in loops:
            calls: List[ MsgData ] = seq.items
            for call in calls:
                labels_dict[ call ] = self.calculateLabel( call )
        return labels_dict

    def _getItemId(self, item_name):
        proper = self.name_dict.get( item_name, None )
        if proper is not None:
            return proper
        name = item_name.replace( "/", "_" )
        self.name_dict[ item_name ] = name
        return name

    def _isToRight(self, from_actor, to_actor):
        from_index = self.actors_order.index( from_actor )
        to_index   = self.actors_order.index( to_actor )
        return from_index < to_index


## ========================================================================


def generate_url( url, label, tooltip ):
    return f"""[[{url} {{{tooltip}}} {label}]]"""


def calculate_actors_optimized_order( graph_actors, labels_dict ) -> List[str]:
#     return sorted( graph_actors )

    distance_dict = {}
    for item, label in labels_dict.items():
        pub = item.pub
        receivers = sorted( item.subs, reverse=True )
        if len( receivers ) < 1:
            ## set non-empty label for first subscriber
            continue
        key = tuple( sorted( [ pub, receivers[0] ] ) )
        distance_dict[ key ] = len( label )

    sorted_actors = list( sorted( graph_actors ) )
    sorted_width  = calculate_width( sorted_actors, distance_dict )
    best_order = sorted_actors
    best_width = sorted_width

    a_size = len( sorted_actors )
    for curr_list in itertools.permutations( sorted_actors, a_size ):
        curr_width = calculate_width( curr_list, distance_dict )
        if curr_width < best_width:
            best_order = list( curr_list )
            best_width = curr_width

    print( "best order:", best_order, best_width, sorted_width )
    return best_order

#     found_best = True
#     while found_best:
#         found_best = False
#         for actor in sorted_actors:
#             index        = best_order.index( actor )
#             combinations = calculate_combinations( best_order, index )
#             for curr_list in combinations:
#                 curr_width = calculate_width( curr_list, distance_dict )
#                 if curr_width < best_width:
#                     best_order = curr_list
#                     best_width = curr_width
#                     found_best = True
#     print( "best order:", best_order, best_width, sorted_width )
#     return best_order


def calculate_width( actors_list, distance_dict ):
    a_size = len( actors_list )
    index_distance = [ 0.0 ] * a_size
    for i in range(1, a_size):
        curr_actor = actors_list[ i ]
        max_dist = 0.0
        for j in range(i - 1, -1, -1):
            prev_actor = actors_list[ j ]
            key  = tuple( sorted( [prev_actor, curr_actor] ) )
            dist = distance_dict.get( key, 0.0 )
            curr_dist = index_distance[ j ] + dist
            max_dist  = max( max_dist, curr_dist )
        index_distance[ i ] = max_dist
    return index_distance[ a_size - 1 ]


def calculate_combinations( actors_list, index ):
    ret_list = []
    list_size = len( actors_list )
    item = actors_list[ index ]
    reduced_list = actors_list.copy()
    del reduced_list[ index ]
    for i in range(0, list_size):
        curr_actor = reduced_list.copy()
        curr_actor.insert( i, item )
        ret_list.append( curr_actor )
    return ret_list


def convert_time_index( index_value ):
    time_unit = "ms"
    time_value = index_value / 1000000     ## in milliseconds
    if time_value > 10000.0:
        time_value = time_value / 1000
        time_unit = "s"
    return ( time_value, time_unit )
