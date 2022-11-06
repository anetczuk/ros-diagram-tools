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

from rosdiagram.graph import Graph


class GraphTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_getNode_string(self):
        graph = Graph()
        graph.addNode( "xxx" )

        node = graph.getNode( "xxx" )
        self.assertTrue( node is not None )

    def test_getNode_slash(self):
        graph = Graph()
        graph.addNode( "/xxx" )

        node = graph.getNode( "/xxx" )
        self.assertTrue( node is not None )

    def test_removeNode_slash(self):
        graph = Graph()
        graph.addNode( "/xxx" )
        self.assertEqual( 1, graph.getNodesCount() )

        removed = graph.removeNode( "/xxx" )
        self.assertTrue( removed )
        self.assertEqual( 0, graph.getNodesCount() )

    def test_removeNode_edges_source(self):
        graph = Graph()
        graph.addEdge( "aaa", "bbb", True )
        self.assertEqual( 2, graph.getNodesCount() )
        self.assertEqual( 1, graph.getEdgesCount() )

        removed = graph.removeNode( "aaa" )
        self.assertTrue( removed )
        self.assertEqual( 1, graph.getNodesCount() )
        self.assertEqual( 0, graph.getEdgesCount() )

    def test_removeNode_edges_dest(self):
        graph = Graph()
        graph.addEdge( "aaa", "bbb", True )
        self.assertEqual( 2, graph.getNodesCount() )
        self.assertEqual( 1, graph.getEdgesCount() )

        removed = graph.removeNode( "bbb" )
        self.assertTrue( removed )
        self.assertEqual( 1, graph.getNodesCount() )
        self.assertEqual( 0, graph.getEdgesCount() )

    def test_detachNodeRaw(self):
        graph = Graph()
        node = graph.addNode( "xxx" )
        self.assertEqual( 1, graph.getNodesCount() )

        removed = graph.detachNodeRaw( node )
        self.assertTrue( removed )
        self.assertEqual( 0, graph.getNodesCount() )

    def test_subgraph_string(self):
        graph = Graph()
        graph.setAsSubgraph()
        content = graph.toString()
        self.assertEqual( content, """{
}
""" )
