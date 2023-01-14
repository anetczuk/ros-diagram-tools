# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from rosdiagram.graphviz import Graph


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def remove_ros_items( graph: Graph ):
    unquoted_names = graph.getNodeNamesAll()
    for name in unquoted_names:
        if name in ( "/rosout", "n_/rosout", "t_/rosout" ):
            graph.removeNode( name )
        if name.startswith( "/rostopic_" ) or name.startswith( "n_/rostopic_" ):
            graph.removeNode( name )
        if name.startswith( "/record_" ) or name.startswith( "n_/record_" ):
            graph.removeNode( name )
