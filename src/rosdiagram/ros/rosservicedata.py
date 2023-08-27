# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import re

from showgraph.io import prepare_filesystem_name, read_file, read_list


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def read_services( service_dir ):
    """
    Read services from dump directory.

    Return dict with following structure:
    { "<service_id>": {                   ## service id
                        "type": str,      ## service type
                       }
      }
    """
    if not service_dir:
        return None
    services_dict = {}
    list_path = os.path.join( service_dir, "list.txt" )
    _LOGGER.debug( "reading services list file: %s", list_path )
    topics_list = read_list( list_path )
    for item in topics_list:
        services_dict[ item ] = read_service( service_dir, item )
    return services_dict


def read_service( services_dump_dir, service_id ):
    if not services_dump_dir or not service_id:
        return None
    item_file = prepare_filesystem_name( service_id )
    item_path = os.path.join( services_dump_dir, item_file + ".txt" )
    if not os.path.isfile( item_path ):
        return None
    item_content = read_file( item_path )
    return parse_content( item_content )


def parse_content( content ):
    msg_type = None

    for line in content.splitlines():
        if len(line) < 1:
            continue

        if "Type:" in line:
            msg_type = match_type( line )
            continue

    deps_dict = {}
    deps_dict['type'] = msg_type
    return deps_dict


def match_type( line ):
    matched = re.findall( r"^Type: (.*)$", line )
    m_size  = len( matched )
    if m_size != 1:
        _LOGGER.warning( "invalid state for line: %s", line )
        return None
    return matched[0]


## ===================================================================


## it happens that topic and node has the same name, so it has to be prefixed
def fix_names( services_dict ):
    label_dict = {}
    all_items = list( services_dict.keys() )

    for item in all_items:
        item_id = "s_" + item
        services_dict[ item_id ] = services_dict.pop( item )
        label_dict[ item_id ] = item

    return label_dict


def get_service_type( services_dict, service_id ) -> str:
    item_data = services_dict.get( service_id, {} )
    return item_data.get( "type", None )
