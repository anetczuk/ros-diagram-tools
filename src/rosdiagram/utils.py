# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

# import os
# import logging
#
#
# SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )
#
# _LOGGER = logging.getLogger(__name__)


def get_create_item( dict_obj, key, default_val ):
    if key not in dict_obj:
        dict_obj[ key ] = default_val
    return dict_obj[ key ]
