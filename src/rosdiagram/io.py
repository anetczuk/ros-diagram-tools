# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import json


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


## read content from file
def read_file( file_path=None ):
    if not os.path.isfile( file_path ):
        return None
    _LOGGER.debug( "loading content from file: %s", file_path )
    with open( file_path, 'r', encoding='utf-8' ) as content_file:
        content = content_file.read()
    return content


def read_list( file_path ):
    if not os.path.isfile( file_path ):
        return []
    ret_list = []
    with open( file_path, 'r', encoding='utf-8' ) as content_file:
        for line in content_file:
            ret_list.append( line.strip() )
    return ret_list


def read_dict( file_path ):
    if not os.path.isfile( file_path ):
        return {}
    with open( file_path, 'r', encoding='utf-8' ) as content_file:
        content = ""
        for line in content_file:
            index = line.find( "#" )
            if index >= 0:
                line = line[ : index ] + "\n"
            content += line
        #content = content_file.read()
        return json.loads( content )
    return {}


## required for JSON to make classes serializable
class CustomJSONEncoder(json.JSONEncoder):
    def default(self, o):
        if hasattr(o, 'toJSON'):
            return o.toJSON()
        raise TypeError(f'Object of type {o.__class__.__name__} is not JSON serializable')


def write_dict( dict_obj, out_file, indent=None ):
    content = json.dumps( dict_obj, indent=indent, cls=CustomJSONEncoder )
    write_file( out_file, content )


def write_file( file_path, content ):
    with open( file_path, 'w', encoding='utf-8' ) as content_file:
        content_file.write( content )


def prepare_filesystem_name( name ):
    new_name = name
    new_name = new_name.replace( "/", "_" )
    new_name = new_name.replace( "|", "_" )
    new_name = new_name.replace( "-", "_" )
    return new_name
