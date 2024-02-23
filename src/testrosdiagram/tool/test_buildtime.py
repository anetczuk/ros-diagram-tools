# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.tool.buildtime import get_build_timestamp


class BuildtimeTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_get_build_timestamp_secs(self):
        timestamp = get_build_timestamp("[build 12.2 s]")
        self.assertEqual( 12.2, timestamp )

    def test_get_build_timestamp_mins(self):
        timestamp = get_build_timestamp("[build 02:12.2 s]")
        self.assertEqual( 132.2, timestamp )

    def test_get_build_timestamp_hrs(self):
        timestamp = get_build_timestamp("[build 1:02:12.2 s]")
        self.assertEqual( 3732.2, timestamp )
