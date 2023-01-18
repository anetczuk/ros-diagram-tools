# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.seqgraph import SequenceGraph, SequenceDetector, MsgData


class MsgDataTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_hashValue(self):
        item1 = MsgData( 0, "aaa", ["bbb"], 0, 0, ["ccc"] )
        val1  = item1.hashValue()
        item2 = MsgData( 1, "aaa", ["bbb"], 0, 0, ["ccc"] )
        val2  = item2.hashValue()
        self.assertEqual( val1, val2 )


## ===========================================================


class SequenceGraphTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_groupCalls(self):
        graph = SequenceGraph()
        graph.addCall( "A", "B", 0, 0, "c1" )
        graph.addCall( "A", "B", 0, 0, "c2" )
        graph.addCall( "A", "B", 0, 0, "c3" )
        self.assertEqual( 3, graph.size() )

        graph.groupCalls()
        self.assertEqual( 1, graph.size() )


## ===========================================================


def detect_string_sequence( string_data: str ):
    seq_detector = SequenceDetector( string_data, lambda data_char: ord(data_char) )       # pylint: disable=W0108
    return seq_detector.detect()


class ModuleTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_detect_sequence_01(self):
        seq = detect_string_sequence( "aabbbcc" )
        self.assertEqual( (2, 1, 3), seq )

    def test_detect_sequence_02(self):
        seq = detect_string_sequence( "aaadededede" )
        self.assertEqual( (3, 2, 4), seq )

    def test_detect_sequence_03(self):
        seq = detect_string_sequence( "aabbbbbbccdedededed" )
        self.assertEqual( (10, 2, 4), seq )

    def test_detect_sequence_04(self):
        seq = detect_string_sequence( "ccdeedeedeedeede" )
        self.assertEqual( (2, 3, 4), seq )
