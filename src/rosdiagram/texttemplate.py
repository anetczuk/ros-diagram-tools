# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
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
