# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import unittest

from rosdiagram.tool.rosindex import configure_parser

import argparse


class ROSIndexTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_configure_parser(self):
        parser = argparse.ArgumentParser()
        configure_parser(parser)
        args = parser.parse_args( [ "--customlist", "aaa", "bbb" ] )
        self.assertEqual( ["aaa", "bbb"], args.customlist )
