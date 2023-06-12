# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import copy
import numpy

from rosdiagram import texttemplate
from rosdiagram.seqgraph import NodeData

# from showgraph.io import write_file, prepare_filesystem_name, read_list

# from rosdiagram.seqgraph import SequenceGraph, SeqItems, MsgData, DiagramData


SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

_LOGGER = logging.getLogger(__name__)


def generate_plantuml_html( output_dir, params_dict=None ):
    msgtypes_dict = params_dict[ "msgtypes_dict" ]

    mainpage_params = params_dict[ "main_page" ]
    mainpage_params[ "msgtypes_dict" ] = msgtypes_dict
    write_seq_main_page( output_dir, mainpage_params )

    nodepages_list = params_dict[ "node_pages" ]
    for nodepage_dict in nodepages_list:
        nodepage_dict[ "msgtypes_dict" ] = msgtypes_dict
        write_seq_node_page( nodepage_dict )

    topicpages_list = params_dict[ "topic_pages" ]
    for topicpage_dict in topicpages_list:
        topicpage_dict[ "msgtypes_dict" ] = msgtypes_dict
        write_seq_node_page( topicpage_dict )

    messagepages_list = params_dict[ "message_pages" ]
    for messagepage_dict in messagepages_list:
        messagepage_dict[ "msgtypes_dict" ] = msgtypes_dict
        write_seq_msg_page( messagepage_dict )

    for msgtype_info in msgtypes_dict.values():
        write_seq_msgtype_page( msgtype_info )


## =====================================================


def write_seq_main_page( output_dir, params_dict: dict ):
    nodes_data    = params_dict['nodes_data']

    main_out_path = os.path.join( output_dir, "full_graph.html" )

    _LOGGER.info( "writing main page: file://%s", main_out_path )
    NodeData.sortList( nodes_data )
    # TopicData.sortList( topics_data )

    template_path = os.path.join( SCRIPT_DIR, "template", "rosbagflow", "baggraph_seq_main_page.html.tmpl" )
    texttemplate.generate( template_path, main_out_path, INPUT_DICT=params_dict )


def write_seq_node_page( params_dict: dict ):
    out_path = params_dict['out_path']

    _LOGGER.info( "writing node page: file://%s", out_path )

    template_path = os.path.join( SCRIPT_DIR, "template", "rosbagflow", "baggraph_seq_node_page.html.tmpl" )
    texttemplate.generate( template_path, out_path, INPUT_DICT=params_dict )


def write_seq_msg_page( params_dict: dict ):
    out_path = params_dict['out_path']
    item     = params_dict['item']

    _LOGGER.info( "writing message page: %s %s %s", out_path, item.index, item.topics )

    template_path = os.path.join( SCRIPT_DIR, "template", "rosbagflow", "baggraph_message.html.tmpl" )
    texttemplate.generate( template_path, out_path, INPUT_DICT=params_dict )


def write_seq_msgtype_page( params_dict: dict ):
    out_path = params_dict['out_path']
    msgtype  = params_dict['msgtype']

    _LOGGER.info( "writing message type page: %s %s", out_path, msgtype )

    template_path = os.path.join( SCRIPT_DIR, "template", "rosbagflow", "baggraph_messagetype.html.tmpl" )
    texttemplate.generate( template_path, out_path, INPUT_DICT=params_dict )


def data_to_dict( data_obj ):
    if isinstance( data_obj, list ):
        return [ data_to_dict( item ) for item in data_obj ]

    if isinstance( data_obj, numpy.ndarray ):
        return list( data_obj )

    try:
        data_dict = None
        if isinstance( data_obj, dict ):
            data_dict = copy.copy( data_obj )
        else:
            data_dict = copy.copy( data_obj.__dict__ )
        for key, val in data_dict.copy().items():
            data_dict[ key ] = data_to_dict( val )

        type_key = "__msgtype__"
        if type_key not in data_dict.keys():
            return data_dict

        ## move '__msgtype__' key to top
        reordered_dict = {}
        reordered_dict[ type_key ] = data_dict[ type_key ]
        del data_dict[ type_key ]
        for key, val in data_dict.items():
            reordered_dict[ key ] = data_to_dict( val )
        return reordered_dict

    # except AttributeError as ex:
    except AttributeError:
        ## object has no attribute '__dict__'
        return data_obj
