# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.datadict import DataDict


class DataDictTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_get(self):
        data = DataDict()
        value = data.get( 'aaa', 'bbb' )
        self.assertEqual( value, "bbb" )

    def test_get_default(self):
        data = DataDict()
        value = data.get( 'aaa', value='xxx' )
        self.assertEqual( value, "xxx" )

    def test_value(self):
        data = DataDict()
        value = data.value( 'aaa', 'bbb' )
        self.assertEqual( value, None )

    def test_value_default(self):
        data = DataDict()
        value = data.value( 'aaa', 'bbb', default='xxx' )
        self.assertEqual( value, "xxx" )
