# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

from rosdiagram.ros.rosparsetools import read_services, read_service        # noqa pylint: disable=W0611


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


SERVICE_PREFIX = "s_"


## it happens that topic and node has the same name, so it has to be prefixed
def fix_names( services_dict ):
    label_dict = {}
    all_items = list( services_dict.keys() )

    for item in all_items:
        item_id = f"{SERVICE_PREFIX}{item}"
        services_dict[ item_id ] = services_dict.pop( item )
        label_dict[ item_id ] = item

    return label_dict


def get_service_type( services_dict, service_id ) -> str:
    item_data = services_dict.get( service_id, {} )
    return item_data.get( "type", None )
