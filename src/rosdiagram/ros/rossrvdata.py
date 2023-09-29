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


def read_srv( srvs_dump_dir, srv_type ):
    if not srvs_dump_dir or not srv_type:
        return None
    item_file = prepare_filesystem_name( srv_type )
    item_path = os.path.join( srvs_dump_dir, item_file + ".txt" )
    if not os.path.isfile( item_path ):
        return None
    item_content = read_file( item_path )
    return item_content


def read_srv_dir( srvs_dump_dir ):
    if not srvs_dump_dir:
        return None
    srvs_dict = {}
    srvs_list_path = os.path.join( srvs_dump_dir, "list.txt" )
    _LOGGER.debug( "reading services list file: %s", srvs_list_path )
    srvs_list = read_list( srvs_list_path )
    for item in srvs_list:
        content = read_srv( srvs_dump_dir, item )
        srvs_dict[ item ] = content
    return srvs_dict
