#!/usr/bin/env python3

import os
import sys
import logging


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

ROS_DIAGRAM_TOOLS_DIR = os.path.join( SCRIPT_DIR, os.pardir, os.pardir, os.pardir, "src" )

sys.path.append( ROS_DIAGRAM_TOOLS_DIR )                     ## allow importing external scripts


from rosdiagram.tool.rosbagflow import main, get_msg_value_name, format_note_error


_LOGGER = logging.getLogger(__name__)


## ===================================================================


def notes_provider_Twist( topics, message_type, message_data ):
    content_lines = []
    linear  = message_data.linear
    angular = message_data.angular
    content = f"linear: [{linear.x}, {linear.y}, {linear.z}] angular: [{angular.x}, {angular.y}, {angular.z}]"
    content_lines.append( content )
    return "\n".join( content_lines )


def notes_provider_TopicStatistics( topics, message_type, message_data ):
    content_lines = []
    content = f"topic '{message_data.topic}' from '{message_data.node_pub}' to '{message_data.node_sub}' delivered: {message_data.delivered_msgs} dropped: {message_data.dropped_msgs}"
    content_lines.append( content )
    return "\n".join( content_lines )


def notes_provider_Log( topics, message_type, message_data ):
    content_lines = []

    header = message_data.header
    content = f"log: '{message_data.name}': \"{message_data.msg}\""

    if not header.frame_id:
        content += " " + format_note_error( f"'header.frame_id' is empty" )

    content_lines.append( content )
    return "\n".join( content_lines )


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
