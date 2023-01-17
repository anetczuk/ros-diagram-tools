# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.graphviz import Graph
import pydotplus


class GraphTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_addNodeObject(self):
        graph = Graph()
        node = pydotplus.Node( "xxx" )
        node.set( "label", "aaa")
        node_obj = graph.addNodeObject( node )
        self.assertEqual( "aaa", node_obj.get("label") )

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

    def test_getNodesByLabels(self):
        graph = Graph()
        node_1: pydotplus.Node = graph.addNode( "xxx1" )
        node_1.set( "label", "aaa" )
        node_2: pydotplus.Node = graph.addNode( "xxx2" )
        node_2.set( "label", "aaa" )
        graph.addNode( "aaa" )

        nodes_list = graph.getNodesByLabels( ["aaa"] )
        self.assertEqual( 3, len(nodes_list) )

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
        self.assertEqual( content, """\
{
}
""" )

    def test_toString_nodes(self):
        graph = Graph()
        graph.addNode( "node_1" )
        graph.addNode( "node_2" )
        graph.addNode( "node_3" )
        graph.addNode( "node_4" )

        content = graph.toString()
        self.assertEqual( """\
digraph G {
node_1;
node_2;
node_3;
node_4;
}
""", content )

    def test_toString_edge(self):
        graph = Graph()
        graph.addEdge( "node_1", "node_2", True )

        content = graph.toString()
        self.assertEqual( """\
digraph G {
node_1;
node_2;
node_1 -> node_2  [color=blue];
}
""", content )
