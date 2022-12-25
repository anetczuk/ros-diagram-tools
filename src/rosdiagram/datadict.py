# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


class DataDict():

    def __init__( self, data_dict=None ):
        self.data = data_dict
        if self.data is None:
            self.data = {}

    def value( self, *keys, default=None ):
        try:
            curr_data = self.data
            for arg in keys:
                curr_data = curr_data[ arg ]
            return curr_data
        except KeyError as exc:
            _LOGGER.warning( "unable to get key %s in %s", exc, keys )
            return default

    def get( self, key, value=None ):
        try:
            return self.data[ key ]
        except KeyError as exc:
            _LOGGER.warning( "unable to get key %s", exc )
            return value
