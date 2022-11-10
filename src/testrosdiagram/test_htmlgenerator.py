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

import unittest

from rosdiagram.htmlgenerator import HtmlGenerator


class HtmlGeneratorTest(unittest.TestCase):
    def setUp(self):
        ## Called before testfunction is executed
        pass

    def tearDown(self):
        ## Called after testfunction was executed
        pass

    def test_getParamNamed_params_None(self):
        params = None
        generator = HtmlGenerator( params_dict=params )
        value = generator._getParamNamed( "node_engine", "xxx" )
        self.assertEqual( None, value )

    def test_getParamNamed_params_empty(self):
        params = {}
        generator = HtmlGenerator( params_dict=params )
        value = generator._getParamNamed( "node_engine", "xxx" )
        self.assertEqual( None, value )

    def test_getParamNamed_None(self):
        params = { "node_engine": None }
        generator = HtmlGenerator( params_dict=params )
        value = generator._getParamNamed( "node_engine", "xxx" )
        self.assertEqual( None, value )

    def test_getParamNamed_value(self):
        params = { "node_engine": "aaa" }
        generator = HtmlGenerator( params_dict=params )
        value = generator._getParamNamed( "node_engine", "xxx" )
        self.assertEqual( "aaa", value )

    def test_getParamNamed_callable(self):
        params = { "node_engine": lambda name: "aaa" }
        generator = HtmlGenerator( params_dict=params )
        value = generator._getParamNamed( "node_engine", "xxx" )
        self.assertEqual( "aaa", value )

    def test_getParamNamed_dict(self):
        params = { "node_engine": { "xxx": "aaa" } }
        generator = HtmlGenerator( params_dict=params )
        value = generator._getParamNamed( "node_engine", "xxx" )
        self.assertEqual( "aaa", value )
