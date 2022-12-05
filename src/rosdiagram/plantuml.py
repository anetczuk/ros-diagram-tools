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

from rosdiagram.io import write_file
from rosdiagram.seqgraph import SequenceGraph, SeqItems


def generate_seq_diagram( seq_graph: SequenceGraph, out_path, group_subs=True ):
    genrator = SequenceDiagramGenerator()
    genrator.generate( seq_graph, out_path, group_subs )


##
class SequenceDiagramGenerator():
    
    def __init__(self):
        self.name_dict = {}

    def generate( self, seq_graph: SequenceGraph, out_path, group_subs=True ):
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

        ## add actors
        actors = seq_graph.actors()
        for item in actors:
            item_id = self._getItemId( item )
            ## content += f"""participant "{item}" as {item_id} [[http://www.google.pl]]\n"""
            content += f"""participant "{item}" as {item_id}\n"""
            
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
    
            loop_content = self.generateLoop( seq, indent, group_subs )
    
            content += loop_content
            if use_msg_loop:
                content += f"""end\n"""
    
        content += "\n@enduml\n"
        
        write_file( out_path, content )

    def generateLoop( self, seq: SeqItems, loop_indent, group_subs=True ):
        content = ""
    
        looped = len( loop_indent ) > 0
    
        calls = seq.items
        for call in calls:
            receivers = call.subs
            
            grouped_topics = len( call.labels ) > 1

            label = " | ".join( call.labels )
            time_unit = "ms"
            time_value = call.index / 1000000     ## in milliseconds
            if time_value > 10000.0:
                time_value = time_value / 1000
                time_unit = "s"
    
            call_label = f"""**{time_value}{time_unit}**: {label}"""
            msg_url    = ""
            indent     = ""
            
            data_url = None
            if grouped_topics or looped:
                ## topics grouped -- no message link
                pass
            else:
                data_url = call.getProp( "url", None )
            if data_url is not None:
                msg_url = f" [[{data_url} message data]]"
            
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

    def _getItemId(self, item_name):
        proper = self.name_dict.get( item_name, None )
        if proper is not None:
            return proper
        name = item_name.replace( "/", "_" )
        self.name_dict[ item_name ] = name
        return name
