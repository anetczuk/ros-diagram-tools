# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from showgraph.io import prepare_filesystem_name, read_file, read_list


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_msg( msgs_dump_dir, msg_type ):
    if not msgs_dump_dir or not msg_type:
        return None
    msg_file    = prepare_filesystem_name( msg_type )
    msg_path    = os.path.join( msgs_dump_dir, msg_file + ".txt" )
    if not os.path.isfile( msg_path ):
        return None
    msg_content = read_file( msg_path )
    return msg_content


def read_msg_dir( msgs_dump_dir ):
    if not msgs_dump_dir:
        return None
    msgs_dict = {}
    msgs_list_path = os.path.join( msgs_dump_dir, "list.txt" )
    _LOGGER.debug( "reading messages list file: %s", msgs_list_path )
    msgs_list = read_list( msgs_list_path )
    for item in msgs_list:
        content = read_msg( msgs_dump_dir, item )
        msgs_dict[ item ] = content
    return msgs_dict
