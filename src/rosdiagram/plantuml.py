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

import os
import datetime
import itertools

from rosdiagram.io import write_file, prepare_filesystem_name
from rosdiagram.seqgraph import SequenceGraph, SeqItems, GraphItem


def generate_seq_diagram( seq_graph: SequenceGraph, out_path, params: dict=None, nodes_subdir="nodes" ):
    genrator = SequenceDiagramGenerator( params )
    genrator.generate( seq_graph, out_path, nodes_subdir )


##
class SequenceDiagramGenerator():
    
    def __init__(self, params: dict=None):
        self.name_dict   = {}
        self.params_dict = params
        if self.params_dict is None:
            self.params_dict = {}

    def generate( self, seq_graph: SequenceGraph, out_path, nodes_subdir="nodes" ):
        call_len = seq_graph.size()
        if call_len < 1:
            content = """\
@startuml
@enduml
"""
            write_file( out_path, content )
            return
    
        content = f"""\
@startuml

skinparam backgroundColor #FEFEFE

"""

        graph_actors = seq_graph.actors()
        labels_dict  = self.calculateLabelsDict( seq_graph )
        actors_order = calculate_actors_optimized_order( graph_actors, labels_dict )

        ## add actors
        for item in actors_order:
            item_id = self._getItemId( item )
#             content += f"""participant "{item}" as {item_id}\n"""

            item_filename = prepare_filesystem_name( item )
            item_path = item_filename + ".html"
            item_path = os.path.join( nodes_subdir, item_path )
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
                content += f"""end\n"""
    
        content += "\n@enduml\n"
        
        write_file( out_path, content )

    def generateLoop( self, seq: SeqItems, loop_indent ):
        content = ""
    
        group_subs = self.params_dict.get( "group_subs", False )
    
        calls = seq.items
        for call in calls:
            receivers = sorted( call.subs, reverse=True )
            
            call_label = self.calculateLabel( call )
            msg_url    = ""
            indent     = ""
            
            if call.isMessageSet():
                data_url = call.getProp( "url", None )
                if data_url is not None:
                    msg_url  = f" [[{data_url} message data]]"
            
            use_subs_group = len( receivers ) > 1 and group_subs
            if use_subs_group:
                ## grouping topic subscribers
                content += f"""{loop_indent}group {call_label}\n"""
                call_label = ""
                indent = "    "
            
            pub_id = self._getItemId( call.pub )
            for rec in receivers:
                rec_id = self._getItemId( rec )
                content   += f"""{loop_indent}{indent}{pub_id} o-> {rec_id} : {call_label}{msg_url}\n"""
                call_label = ""     ## clear label after first item
                msg_url    = ""

            if use_subs_group:
                content += f"""{loop_indent}end\n"""
    
        return content

    def calculateLabel(self, item: GraphItem ):
        label            = " | ".join( item.labels )
        timestamp_dt     = datetime.datetime.fromtimestamp( item.timestamp / 1000000000 )
        timestamp_string = timestamp_dt.strftime('%H:%M:%S.%f')
        call_label       = f"""**{timestamp_string}**: {label}"""
        return call_label

    def calculateLabelsDict(self, seq_graph: SequenceGraph ):
        labels_dict = {}
        loops: List[ SeqItems ] = seq_graph.getLoops()
        for seq in loops:
            calls: List[ GraphItem ] = seq.items
            for call in calls:
                labels_dict[ call ] = self.calculateLabel( call ) + " message data"
        return labels_dict

    def _getItemId(self, item_name):
        proper = self.name_dict.get( item_name, None )
        if proper is not None:
            return proper
        name = item_name.replace( "/", "_" )
        self.name_dict[ item_name ] = name
        return name


## ========================================================================


def calculate_actors_optimized_order( graph_actors, labels_dict ):
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
            best_order = curr_list
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
        for j in range(i-1, -1, -1):
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
