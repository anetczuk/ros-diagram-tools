# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree. 
#

import unittest

from rosdiagram.htmlgenerator import ParamsDict


class ParamsDictTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_getNamed_params_None(self):
        params = None
        params_dict = ParamsDict( params=params )
        value = params_dict.getNamed( "node_engine", "xxx" )
        self.assertEqual( None, value )

    def test_getNamed_params_empty(self):
        params = {}
        params_dict = ParamsDict( params=params )
        value = params_dict.getNamed( "node_engine", "xxx" )
        self.assertEqual( None, value )

    def test_getNamed_None(self):
        params = { "node_engine": None }
        params_dict = ParamsDict( params=params )
        value = params_dict.getNamed( "node_engine", "xxx" )
        self.assertEqual( None, value )

    def test_getNamed_value(self):
        params = { "node_engine": "aaa" }
        params_dict = ParamsDict( params=params )
        value = params_dict.getNamed( "node_engine", "xxx" )
        self.assertEqual( "aaa", value )

    def test_getNamed_callable(self):
        params = { "node_engine": lambda name: "aaa" }
        params_dict = ParamsDict( params=params )
        value = params_dict.getNamed( "node_engine", "xxx" )
        self.assertEqual( "aaa", value )

    def test_getNamed_dict(self):
        params = { "node_engine": { "xxx": "aaa" } }
        params_dict = ParamsDict( params=params )
        value = params_dict.getNamed( "node_engine", "xxx" )
        self.assertEqual( "aaa", value )
