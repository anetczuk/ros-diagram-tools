# MIT License
#
# Copyright (c) 2022 Arkadiusz Netczuk <dev.arnet@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
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
