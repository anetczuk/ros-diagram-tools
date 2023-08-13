#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import sys
import os
import logging


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

SRC_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir, os.pardir, "src"))
sys.path.insert(0, SRC_DIR)


from showgraph.graphviz import Graph

from rosdiagram.graphviztohtml import generate_from_template


_LOGGER = logging.getLogger(__name__)


## =============================================================


def main():
    _LOGGER.info( "data generation" )

    dot_graph = Graph()
    dot_graph.setName( "simple_graph" )
    dot_graph.addEdge( "Node #1", "Node #2", create_nodes=True )

    main_dict = {   "page_title": "sample dotgraph page",
                    "main_page_link": "",
                    "top_content": "<br/>Example of graph in general page template.",
                    "graph": dot_graph,
                    "items_lists": [ { "title": "Example list of items with links",
                                       "items": [ ("www.google.com", ""),
                                                  ("www.wikipedia.com", "http://www.wikipedia.com"),
                                                  ("simple graph", "simple_graph.png")
                                                ] }
                                   ],
                    "bottom_content": "Example of general bottom content with <b>HTML</b>."
                }

    out_dir = os.path.join( SCRIPT_DIR, "out" )
    template = "dotgraph_page.html"
    generate_from_template( out_dir, main_dict, template_name=template )


if __name__ == '__main__':
    main()
