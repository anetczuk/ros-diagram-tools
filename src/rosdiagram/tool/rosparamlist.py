# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse
import html
import json
import yaml

from showgraph.io import prepare_filesystem_name, write_file

from rosdiagram.graphviztohtml import generate_from_template


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

DATA_SUBDIR = "data"


## ===================================================================


def generate_pages( params_dict, out_dir, outhtml, outmarkdown ):
    os.makedirs( out_dir, exist_ok=True )

    data_dir = os.path.join( out_dir, DATA_SUBDIR )
    os.makedirs( data_dir, exist_ok=True )

    fatten_list = flatten_dict( params_dict )

    fatten_list.sort( key=lambda item: item[0], reverse=False )
    params_list = prepare_params_list(fatten_list, out_dir, DATA_SUBDIR)

    main_dict = {   "style": {},
                    "params_list": params_list
                    }

    if outhtml:
        template = "rosparam.html"
        generate_from_template( out_dir, main_dict, template_name=template )

    if outmarkdown:
        template = "rosparam.md"
        generate_from_template( out_dir, main_dict, template_name=template )


def flatten_dict( d, parent=""):
    ret_list = []
    for k, v in d.items():
        flat_key = parent + "/" + k
        if isinstance(v, dict):
            ret_sublist = flatten_dict( v, flat_key )
            ret_list.extend( ret_sublist )
        else:
            ret_list.append( (flat_key, v) )
    return ret_list


def find_nth_index( haystack, needle, n ):
    start = haystack.find(needle)
    while start >= 0 and n > 1:
        start = haystack.find(needle, start + 1)
        n -= 1
    return start


# params_list: [ (<param-name>, <param-value>) ]
def prepare_params_list(params_data_list, out_dir, out_subdir):
    params_list = []
    for item in params_data_list:
        param = item[0]
        value = item[1]
        value_type = type(value).__name__

        data_subpath  = prepare_filesystem_name( param )
        data_subpath  = os.path.join( out_subdir, f"{data_subpath}.txt" )
        data_fullpath = os.path.join( out_dir, data_subpath )
        write_file( data_fullpath, str(value) )

        value_str = convert_to_html(value)

        params_list.append( (param, data_subpath, value_type, value_str) )

    return params_list


def convert_to_html(value):
    value_str = ""
    if isinstance( value, (dict, list) ):
        # try to dump by JSON
        value_str = json.dumps( value, indent=4 )
    else:
        value_str = str(value)

    if len(value_str) > 1024 or value_str.count('\n') > 20:
        pos = find_nth_index( value_str, '\n', 20 )
        pos = min( pos, 1020 )
        value_str = value_str[:pos] + "\n..."

    value_str = html.escape( value_str )
    return value_str


## ===================================================================


def configure_parser( parser ):
    parser.description = 'rosparam parameters list'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--dumpyamlfile', action='store', required=True,
                         help="Path to rosparam dump file" )
    parser.add_argument( '--outhtml', action='store_true', help='Output HTML' )
    parser.add_argument( '--outmarkdown', action='store_true', help='Output Markdown' )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output directory" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    with open( args.dumpyamlfile, 'r', encoding='utf-8' ) as content_file:
        params_dict = yaml.safe_load( content_file )

    ##
    ## generate data
    ##
    if (args.outhtml or args.outmarkdown) and args.outdir:
        _LOGGER.info( "generating graphs" )
        generate_pages( params_dict, args.outdir, args.outhtml, args.outmarkdown )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
