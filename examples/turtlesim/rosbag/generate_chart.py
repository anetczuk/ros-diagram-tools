#!/usr/bin/env python3

import os
import sys
import logging


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

ROS_DIAGRAM_TOOLS_DIR = os.path.join( SCRIPT_DIR, os.pardir, os.pardir, os.pardir, "src" )

sys.path.append( ROS_DIAGRAM_TOOLS_DIR )                     ## allow importing external scripts


from rosdiagram.tool.rosbagflow import main, get_msg_value_name, get_msg_value_enum, NotesContainer


_LOGGER = logging.getLogger(__name__)


## ===================================================================


def notes_provider_Twist( topics, message_type, message_data ):
    container = NotesContainer()
    linear  = message_data.linear
    angular = message_data.angular
    container.addInfo( f"linear: [{linear.x}, {linear.y}, {linear.z}]" )
    container.addInfo( f"angular: [{angular.x}, {angular.y}, {angular.z}]" )
    return container


def notes_provider_TopicStatistics( topics, message_type, message_data ):
    container = NotesContainer( bg_color="#FFBBBB" )
    container.addInfo( f"topic '{message_data.topic}'" )
    container.addInfo( f"from '{message_data.node_pub}'" )
    container.addInfo( f"to '{message_data.node_sub}'" )
    container.addInfo( f"delivered: {message_data.delivered_msgs}" )
    container.addInfo( f"dropped: {message_data.dropped_msgs}" )
    return container


def notes_provider_Log( topics, message_type, message_data ):
    container = NotesContainer( topics, "aqua" )
    header    = message_data.header

    level_data = get_msg_value_enum( message_data, 'level' )
    container.addInfoEnum( "level:", *level_data )

    container.addInfo( f"log: '{message_data.name}': '{message_data.msg}'" )

    if not header.frame_id:
        container.addError( f"'header.frame_id' is empty" )
        #content += " " + format_note_error(  )
    return container


## ==========================================================


NOTES_DICT = { "geometry_msgs/msg/Twist": notes_provider_Twist,
               "rosgraph_msgs/msg/TopicStatistics": notes_provider_TopicStatistics,
               "rosgraph_msgs/msg/Log": notes_provider_Log
               }


def notes_provider( topics, message_type, message_data ):
    provider = NOTES_DICT.get( message_type )
    if provider is None:
        return None
    return provider( topics, message_type, message_data )


## ============================= main section ===================================


if __name__ == '__main__':
    main( notes_functor=notes_provider )
