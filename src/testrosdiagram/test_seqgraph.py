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

import unittest

from rosdiagram.seqgraph import SequenceGraph, SequenceDetector, GraphItem


class GraphItemTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_hashValue(self):
        item1 = GraphItem( "aaa", ["bbb"], 0, 0, ["ccc"] )
        val1  = item1.hashValue()
        item2 = GraphItem( "aaa", ["bbb"], 0, 0, ["ccc"] )
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
    hash_function = lambda data_char: ord(data_char)
    seq_detector  = SequenceDetector( string_data, hash_function )
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
