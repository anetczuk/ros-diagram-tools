# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from rosdiagram.io import prepare_filesystem_name, read_file


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
