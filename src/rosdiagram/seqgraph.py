# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import copy
from dataclasses import dataclass, field
from typing import Set, List, Any, Dict


##
class MsgData():

    def __init__(self, pub: str, subs: Set[str], index: int, timestamp, labels: Set[str] ):
        self.pub       = pub
        self.subs      = subs
        self.index     = index
        self.timestamp = timestamp
        self.labels    = labels

        self.msgtype = None
        self.msgdef  = None
        self.msgdata = None

        self.notes_data = None

        self.props: Dict[ str, Any ] = {}

    def copy(self):
        return copy.deepcopy( self )

    def setMessageData( self, msgtype, msgdef, msgdata ):
        self.msgtype = msgtype
        self.msgdef  = msgdef
        self.msgdata = msgdata

    def clearMessageaData(self):
        self.msgtype = None
        self.msgdef  = None
        self.msgdata = None

    def isMessageSet(self):
        if self.msgtype is None:
            return False
        if self.msgdef is None:
            return False
        if self.msgdata is None:
            return False
        return True

    def getProp(self, key, def_value=None):
        return self.props.get( key, def_value )

    def setProp(self, key, value):
        self.props[ key ] = value

    def haveActor(self, actor):
        if self.pub == actor:
            return True
        if actor in self.subs:
            return True
        return False

    def haveLabel(self, label):
        return label in self.labels

    def sameActors( self, other_item: 'MsgData' ):
        if self.pub != other_item.pub:
            return False
        if self.subs != other_item.subs:
            return False
        return True

    def sameLabels( self, other_item: 'MsgData' ):
        return self.labels == other_item.labels

    def addLabels(self, labels: Set[str] ):
        self.labels = self.labels.union( labels )

    def hashValue(self):
        string_list = []
        string_list.append( self.pub )
        string_list.extend( self.subs )
        string_list.extend( self.labels )
        return hash( tuple( string_list ) )


##
class SeqItems():

    def __init__( self, items, repeat=1 ):
        self.items: List[ MsgData ] = items
        self.repeats: int             = repeat
        if repeat > 1:
            for item in self.items:
                item.clearMessageaData()

    def size(self):
        return len( self.items )


## =============================================================


##
class SequenceGraph():

    def __init__(self):
        self.callings: List[ MsgData ] = []
        self.loops: List[ SeqItems ]     = []

    def size(self):
        return len( self.callings )

    def itemsNum(self):
        if len( self.loops ) < 1:
            return len( self.callings )
        counter = 0
        for seq in self.loops:
            counter += seq.size()
        return counter

    def getLoops(self) -> List[ SeqItems ]:
        if len( self.loops ) < 1:
            self.loops.append( SeqItems( self.callings ) )
        return self.loops

    def loopsFound(self):
        for loop in self.loops:
            if loop.repeats > 1:
                return True
        return False

    def actors(self) -> Set[str]:
        ret_set = set()
        for calls in self.callings:
            ret_set.add( calls.pub )
            ret_set.update( calls.subs )
        return ret_set

    def addCall(self, publisher, subscriber, index, timestamp, label) -> MsgData:
        item = MsgData( publisher, set(subscriber,), index, timestamp, set([label]) )
        self.callings.append( item )
        return item

    def addCallSubs(self, publisher, subscribers_list, index, timestamp, label) -> MsgData:
        item = MsgData( publisher, set(subscribers_list), index, timestamp, set([label]) )
        self.callings.append( item )
        return item

    def process(self, params: dict = None ):
        if params is None:
            params = {}

        if params.get( "group_calls", False ):
            self.groupTopics()

        if params.get( "group_topics", False ):
            self.groupCalls()

        if params.get( "detect_loops", False ):
            self.zipSeqs()

    def groupCalls(self):
        call_len = len(self.callings)
        if call_len < 2:
            return self.callings
        groups = []
        prev_call = self.callings[0]
        for i in range(1, call_len ):
            call = self.callings[i]
            if prev_call.sameActors( call ):
                ## same pub and subs
                prev_call.addLabels( call.labels )
                prev_call.clearMessageaData()
                continue
            groups.append( prev_call )
            prev_call = call
        groups.append( prev_call )
        self.callings = groups
        return self.callings

    def groupTopics(self):
        call_len = len(self.callings)
        if call_len < 2:
            return self.callings
        groups = []
        prev_call = self.callings[0]
        for i in range(1, call_len ):
            call = self.callings[i]
            if prev_call.sameActors( call ) and prev_call.sameLabels( call ):
                prev_call.clearMessageaData()
                continue
            groups.append( prev_call )
            prev_call = call
        groups.append( prev_call )
        self.callings = groups
        return self.callings

    def zipSeqs(self):
        improved = True
        curr_loops: List[ SeqItems ] = self.getLoops()
        while improved:
            improved  = False
            new_loops: List[ SeqItems ] = []
            for items_seq in curr_loops:
                if items_seq.repeats > 1:
                    new_loops.append( items_seq )
                    continue
                callings = items_seq.items
                seq_detector = SequenceDetector( callings, lambda item: item.hashValue() )
                best_seq     = seq_detector.detect()
                seq_gain     = calculate_seq_gain( best_seq )
                if seq_gain < 2:
                    new_loops.append( items_seq )
                    continue

                print( "best loop detected:", best_seq )
                improved = True
                start_index = best_seq[0]
                next_index  = best_seq[0] + best_seq[1]
                after_index = best_seq[0] + best_seq[1] * best_seq[2]

                prev = callings[ : start_index ]
                if len( prev ) > 0:
                    new_loops.append( SeqItems( prev ) )

                repeat = callings[ start_index: next_index ]
                new_loops.append( SeqItems( repeat, best_seq[2] ) )

                after = callings[ after_index: ]
                if len( after ) > 0:
                    new_loops.append( SeqItems( after ) )
            curr_loops = new_loops
        self.loops = curr_loops

    def copyCallingsActors(self, actor ):
        new_calls: List[ MsgData ] = []
        for call in self.callings:
            if call.haveActor( actor ):
                new_call = call.copy()
                new_calls.append( new_call )
        graph = SequenceGraph()
        graph.callings = new_calls
        return graph

    def copyCallingsLabels(self, label ):
        new_calls: List[ MsgData ] = []
        for call in self.callings:
            if call.haveLabel( label ):
                new_call = call.copy()
                new_calls.append( new_call )
        graph = SequenceGraph()
        graph.callings = new_calls
        return graph


## ===================================================================


@dataclass
class NodeData():
    name: str       = None
    suburl: str     = None
    excluded: bool  = False

    def __getitem__(self, key):
        if key == 0:
            return self.name
        if key == 1:
            return self.suburl
        if key == 2:
            return self.excluded
        raise IndexError( f"invalid index: {key}" )


@dataclass
class TopicData():
    name: str       = None
    msgcount: int   = 0
    excluded: bool  = False
    suburl: str     = None

    def __getitem__(self, key):
        if key == 0:
            return self.name
        if key == 1:
            return self.msgcount
        if key == 2:
            return self.excluded
        raise IndexError( f"invalid index: {key}" )


@dataclass
class DiagramData():
    seq_diagram: SequenceGraph  = None
    params: Dict[ str, Any ]    = field(default_factory=lambda: {})
    nodes: List[ NodeData ]     = field(default_factory=lambda: [])
    topics: List[ TopicData ]   = field(default_factory=lambda: [])

    nodes_subdir                = "nodes"
    topics_subdir               = "topics"
    msgs_subdir                 = "msgs"


## =============================================================


def detect_sequence( data_list, item_hash_function ):
    seq_detector = SequenceDetector( data_list, item_hash_function )
    return seq_detector.detect()


##
class SequenceDetector():

    def __init__( self, data_list, item_hash_function ):
        self.hash_list = []
        for item in data_list:
            value = item_hash_function( item )
            self.hash_list.append( value )
        self.list_len = len( self.hash_list )

        self.best_seq  = [0, 0, 0]
        self.best_gain = 0

    def detect(self):
        self.best_seq  = [0, 0, 0]
        self.best_gain = 0
        max_seq_len = min( 100, self.list_len )
        for seq_len in range( 1, max_seq_len ):
            seq      = self.detectSeq( seq_len )
            seq_gain = calculate_seq_gain( seq )
            if seq_gain > self.best_gain:
                self.best_seq  = seq
                self.best_gain = seq_gain
        return self.best_seq

    def detectSeq(self, seq_len):
        seq_max  = self.list_len - seq_len + 1
        best_seq = [0, 0, 0]
        for start_index in range( 0, self.list_len ):
            pot_repeat = (self.list_len - start_index) / seq_len
            pot_gain   = calculate_seq_gain( [start_index, seq_len, pot_repeat] )
            if pot_gain < self.best_gain:
                break
            seq_repeat = 1
            for next_index in range( start_index + seq_len, seq_max, seq_len ):
                if self.compareSeq( start_index, next_index, seq_len ) is False:
                    break
                seq_repeat += 1
            if seq_repeat > best_seq[2]:
                best_seq = (start_index, seq_len, seq_repeat)
        return best_seq

    def compareSeq(self, first_index, second_index, seq_len):
        for i in range( 0, seq_len ):
            if self.hash_list[ first_index + i ] != self.hash_list[ second_index + i ]:
                return False
        return True


def calculate_seq_gain( seq ):
    seq_length = seq[1] * seq[2]
    return seq_length - seq[1]
