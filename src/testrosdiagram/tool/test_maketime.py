# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.tool.maketime import get_after


class MaketimeTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_get_after(self):
        content = get_after( "fffaaaccc", "aaa")
        self.assertEqual( "ccc", content )

    def test_get_after_last(self):
        content = get_after( "fffaaa", "aaa")
        self.assertEqual( "", content )

    def test_get_after_not_found(self):
        content = get_after( "fffaaa", "bbb")
        self.assertEqual( None, content )

    def test_get_after_regex(self):
        content = get_after( "fffaaaccc", "a+")
        self.assertEqual( "ccc", content )
