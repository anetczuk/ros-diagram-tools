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

import texthon
import texthon.parser


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


## read content from file
def generate( template_path, output_path, *args, **kwargs ):
    engine = texthon.Engine()

    parser = texthon.parser.Parser()
    module_def = engine.load_file( template_path, parser=parser )       # parse and store the parsed module

    # store the path so we can find the compiled module later
    module_id  = module_def.path

    engine.make()                                                       # compile all modules

    module = engine.modules[module_id]

    # call the template function named 'main'
    script_content = module.main( *args, **kwargs )

    ### === writing to file ===
    with open( output_path, "w", encoding='utf-8' ) as out_file:
        out_file.write( script_content )
