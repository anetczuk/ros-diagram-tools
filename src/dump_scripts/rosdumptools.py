#!/usr/bin/env python3
#
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

#
# Keep this file as independent as possible (without dependencies), because it will be run on
# ROS devices to retrieve ROS data.
#

import sys
import os
import logging
import argparse
import pprint
import json
import typing
from collections.abc import Iterable

import rospkg
import rosnode
import rostopic
import rosservice
import rosmsg
import roslaunch


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## =====================================================


def read_list( file_path ):
    if not os.path.isfile( file_path ):
        return []
    ret_list = []
    with open( file_path, 'r', encoding='utf-8' ) as content_file:
        for line in content_file:
            ret_list.append( line.strip() )
    return ret_list


def write_list( file_path, dump_list ):
    with open(file_path, 'w', encoding='utf8' ) as fp:
        fp.write('\n'.join(dump_list))


def prepare_filesystem_name( name ):
    new_name = name
    new_name = new_name.replace( "/", "_" )
    new_name = new_name.replace( "|", "_" )
    new_name = new_name.replace( "-", "_" )
    return new_name


## =====================================================


def rospack_configure( parser ):
    parser.description = 'dump rospack data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rospack_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    rp = rospkg.RosPack()
    packages = rp.list()
    packages = sorted( packages )

    out_list = os.path.join( out_dir, "list.txt" )

    with open(out_list, 'w', encoding='utf8' ) as fp:
        for item in packages:
            pkg_path = rp.get_path(item)
            fp.write( f'{item} {pkg_path}\n' )

            depends1 = rp.get_depends(item, implicit=False)
            out_pkg = os.path.join( out_dir, f"{item}.txt" )
            write_list( out_pkg, depends1 )

    print( "Done." )


## =====================================================


def rosnode_configure( parser ):
    parser.description = 'dump rosnode data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rosnode_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    nodes_list = rosnode.get_node_names()
    nodes_list = sorted( nodes_list )

    out_list = os.path.join( out_dir, "list.txt" )

    write_list( out_list, nodes_list )

    for item in nodes_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            node_info = rosnode.get_node_info_description(item)
            fp.write( node_info )

    print( "Done." )


## =====================================================


def rostopic_configure( parser ):
    parser.description = 'dump rostopic data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rostopic_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    publishers, subscribers = rostopic.get_topic_list()
    topics_list = set()
    topics_list.update( [ pub[0] for pub in publishers ] )
    topics_list.update( [ sub[0] for sub in subscribers ] )
    topics_list = sorted( list( topics_list ) )

    out_list = os.path.join( out_dir, "list.txt" )

    write_list( out_list, topics_list )

    for item in topics_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            topic_info = rostopic.get_info_text( item )
            fp.write( topic_info )

    print( "Done." )


## =====================================================


def rosservice_configure( parser ):
    parser.description = 'dump rosservice data'
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rosservice_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    print( f"Dumping data to {out_dir}" )

    services_list = rosservice.get_service_list()
    services_list = sorted( list(services_list) )

    out_list = os.path.join( out_dir, "list.txt" )

    write_list( out_list, services_list )

    for item in services_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        service_node = rosservice.get_service_node( item )
        service_uri = rosservice.get_service_uri( item )
        service_type = rosservice.get_service_type( item )
        service_args = rosservice.get_service_args( item )

        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            out_data = f"""\
Node: {service_node}
URI: {service_uri}
Type: {service_type}
Args: {service_args}
"""
            fp.write( out_data )

    print( "Done." )


## =====================================================


def rosmsg_configure( parser ):
    parser.description = 'dump rosmsg data'
    parser.add_argument( '--listprovided', action='store_true', required=False, default="",
                         help="Read list of messages from list file" )
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rosmsg_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    out_list = os.path.join( out_dir, "list.txt" )

    print( f"Dumping data to {out_dir}" )

    rp = rospkg.RosPack()
    packages = rp.list()
    packages = sorted( packages )

    messages_list = set()
    if args.listprovided:
        # read items from list
        messages_list = read_list( out_list )
    else:
        for pkg in packages:
            msg_list = rosmsg.list_msgs( pkg, rp )
            if msg_list:
                messages_list.update( msg_list )
        messages_list = sorted( list( messages_list ) )
        write_list( out_list, messages_list )

    for item in messages_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        msg_info = rosmsg.get_msg_text( item, rospack=rp )

        print( f"Writing {out_info_path}" )
        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            fp.write( msg_info )

    print( "Done." )


## =====================================================


def rossrv_configure( parser ):
    parser.description = 'dump rossrv data'
    parser.add_argument( '--listprovided', action='store_true', required=False, default="",
                         help="Read list of messages from list file" )
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output dir" )


def rossrv_process( args ):
    out_dir = args.outdir
    os.makedirs(out_dir, exist_ok=True)

    out_list = os.path.join( out_dir, "list.txt" )

    print( f"Dumping data to {out_dir}" )

    rp = rospkg.RosPack()
    packages = rp.list()
    packages = sorted( packages )

    services_list = set()
    if args.listprovided:
        # read items from list
        services_list = read_list( out_list )
    else:
        for pkg in packages:
            srv_list = rosmsg.list_srvs( pkg, rp )
            if srv_list:
                services_list.update( srv_list )
        services_list = sorted( list( services_list ) )
        write_list( out_list, services_list )

    for item in services_list:
        item_filename = prepare_filesystem_name( item )
        out_info_path = os.path.join( out_dir, f"{item_filename}.txt" )

        srv_info = rosmsg.get_srv_text( item, rospack=rp )

        print( f"Writing {out_info_path}" )
        with open(out_info_path, 'w', encoding='utf8' ) as fp:
            fp.write( srv_info )

    print( "Done." )


## =====================================================


class ObjRepr:

    def __init__(self):
        self._visited = set()

    def reprObj(self, obj):
        self._visited.clear()
        return self._visit(obj)

    def _visit(self, obj):
        obj_id = id(obj)
        if obj_id in self._visited:
            # print("visited:", type(next_obj), next_obj)
            return obj
        self._visited.add(obj_id)

        if isinstance(obj, dict):
            ret_dict = {}
            for key, data in obj.items():
                ret_dict[key] = self._visit(data)
            return ret_dict

        if hasattr(obj, "__dict__"):
            ret_dict = {"___type___": type(obj).__name__, "___id___": id(obj)}
            for key, data in obj.__dict__.items():
                ret_dict[key] = self._visit(data)
            return ret_dict

        if hasattr(obj, "__slots__"):
            ret_dict = {"___type___": type(obj).__name__, "___id___": id(obj)}
            for key in obj.__slots__:
                data = getattr(obj, key)
                ret_dict[key] = self._visit(data)
            return ret_dict

        if isinstance(obj, str):
            return obj

        if isinstance(obj, Iterable):
            ret_list = []
            for data in obj:
                ret_list.append( self._visit(data) )
            return ret_list

        return obj


def obj_to_dict(obj):
    repr_obj = ObjRepr()
    return repr_obj.reprObj(obj)


## =====================================================


def roslaunch_configure( parser ):
    parser.description = 'dump roslaunch data'
    # parser.add_argument( '--listprovided', action='store_true', required=False, default="",
    #                      help="Read list of messages from list file" )
    parser.add_argument( '--launchfile', action='store', required=True, default="",
                         help="Path to launch file" )
    parser.add_argument( '--outdir', action='store', required=True, default="",
                         help="Path to output directory" )


## included files (ROSLaunchConfig.roslaunch_files)
## nodes (ROSLaunchConfig.nodes)
## parameters (ROSLaunchConfig.params)
## store struct: LaunchDumpData
def roslaunch_process( args ):
    launch_path = args.launchfile

    master_dict = load_launch(launch_path)
    loader_master: LaunchLoader = master_dict["loader"]

    # pprint.pprint( obj_to_dict(loader_master) )

    launch_tree = LanuchDataTree()
    launch_tree.set_by_lunch_dict(master_dict)

    for child_ctx in loader_master.root_context.children:
        generate_launch_tree( child_ctx, launch_tree )

    # pprint.pprint( obj_to_dict(launch_tree) )

    dump_data = launch_tree.get_dump_data()
    output_data = obj_to_dict(dump_data)
    # pprint.pprint( output_data, sort_dicts=False )

    launch_out_file = os.path.join(args.outdir, "launch.json")
    launch_out_file = os.path.abspath(launch_out_file)
    print( f"Writing {launch_out_file}" )
    with open(launch_out_file, 'w', encoding='utf8' ) as fp:
        json.dump(output_data, fp, indent=4)

    print( "Done." )


# =========================================================


class LaunchDumpData:
    def __init__(self):
        self.file: str = None
        self.launch_args = []                               # LoaderContext.args_passed
        self.resolve_dict: {} = None                        # LoaderContext.resolve_dict
        self.nodes = None                                   # ROSLaunchConfig.nodes
        self.params = None                                  # ROSLaunchConfig.params
        # self.machines = None                                # ROSLaunchConfig.machines
        self.included: typing.List[LaunchDumpData] = []

    def append(self, item):
        self.included.append(item)


class LanuchDataTree:
    def __init__(self, context=None, config=None):
        self.context = context                                # LoaderContext
        self.config = config                                  # ROSLaunchConfig
        self.included: typing.List[LanuchDataTree] = []

    def is_empty(self):
        return len(self.included) < 1

    def set_by_lunch_dict(self, data_dict):
        self.context = data_dict["loader"].root_context
        self.config = data_dict["config"]

    def append(self, data_tree):
        self.included.append(data_tree)

    def append_lunch_dict(self, context, config):
        item = LanuchDataTree()
        item.context = context
        item.config = config
        self.included.append(item)

    def add_lunch_dict(self, parent_id, context, config) -> bool:
        parent_item = self.get_item_by_contex_id(parent_id)
        if parent_item is None:
            _LOGGER.warning("unable to find item by id: %s %s", parent_id, context.filename)
            return False
        parent_item.append_lunch_dict(context, config)
        return True

    def get_item_by_contex_id(self, context_id):
        if self.is_match_context_id(context_id):
            return self
        for item in self.included:
            found = item.get_item_by_contex_id(context_id)
            if found:
                return found
        return None

    def is_match_context_id(self, context_id):
        return id(self.context) == context_id

    def get_dump_data(self) -> LaunchDumpData:
        ret_item = LaunchDumpData()
        ret_item.file: str = self.config.roslaunch_files[0]
        if hasattr(self.context, "args_passed"):
            ret_item.launch_args = self.context.args_passed
        ret_item.resolve_dict = self.context.resolve_dict
        ret_item.nodes = self.config.nodes
        ret_item.params = self.config.params
        # ret_item.machines = self.config.machines

        for item in self.included:
            dump_item: LaunchDumpData = item.get_dump_data()
            ret_item.append(dump_item)
        return ret_item


def generate_launch_tree(context, parent_tree: LanuchDataTree):
    if not hasattr(context, "children"):
        # node case
        return

    parent = parent_tree
    if context.tag_name == "include":
        child_dict = load_launch_by_context(context)
        conf = child_dict["config"]
        child_tree = LanuchDataTree(context, conf)
        parent.append(child_tree)
        parent = child_tree

    for child_ctx in context.children:
        generate_launch_tree( child_ctx, parent )


class LaunchLoader(roslaunch.xmlloader.XmlLoader):

    def __init__(self):
        super().__init__()
        self.ignore_unset_args = False    # workaround for XmlLoader crash (missing attribute)

        self.all_children = []

    def _ns_clear_params_attr(self, tag_name, tag, context, ros_config, node_name=None, include_filename=None):
        child_context = super()._ns_clear_params_attr( tag_name, tag, context, ros_config, node_name, include_filename )
        child_context.tag_name = tag_name
        self.all_children.append(child_context)
        return child_context

    def build_include_tree(self):
        for child in self.all_children:
            if not hasattr(child.parent, "children"):
                child.parent.children = []
            child.parent.children.append(child)


def load_launch(launch_path, argv=None):
    _LOGGER.info("parsing launch file: %s %s", launch_path, argv)
    config = roslaunch.config.ROSLaunchConfig()
    loader = LaunchLoader()
    # loader = roslaunch.xmlloader.XmlLoader()

    # loader -- contains input data for launch file
    # config -- contains result data of launch file
    loader.load(launch_path, config, argv=argv, verbose=False)
    loader.build_include_tree()

    config.logger = None
    config.machines = None                              # causes serialization problems
    for child in loader.all_children:
        child.parent_id = id(child.parent)
        child.parent = None                             # causes serialization problems
    del loader.all_children

    pprint.pprint( obj_to_dict(loader) )

    return {"file": os.path.abspath(launch_path),
            "loader": loader,
            "config": config}


def load_launch_by_context(context):
    launch_file = context.filename
    context_args = get_passed_args(context)
    return load_launch(launch_file, context_args)


def get_passed_args(context: roslaunch.loader.LoaderContext):
    if not hasattr(context, "args_passed"):
        return []
    ret_list = []
    passed_args = context.args_passed
    resolved_args = context.resolve_dict.get("arg", {})
    for passed in passed_args:
        value = resolved_args[passed]
        ret_list.append(f"{passed}:={value}")
    return ret_list


def get_data_from_config(config: roslaunch.config.ROSLaunchConfig):
    ret_dict = { "file": config.roslaunch_files[0],
                 "nodes": obj_to_dict(config.nodes),
                 "params": obj_to_dict(config.params),
                 "machines": obj_to_dict(config.machines)
                 }
    return ret_dict


## =====================================================


def main():
    parser = argparse.ArgumentParser(description='ROS parse tools')
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    parser.add_argument( '--listtools', action='store_true', help='List tools' )

    subparsers = parser.add_subparsers( help='one of tools', description="use one of tools",
                                        dest='tool', required=False )

    ## =================================================

    subparser = subparsers.add_parser('rospack', help='dump rospack data')
    subparser.set_defaults( func=rospack_process )
    rospack_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosnode', help='dump rosnode data')
    subparser.set_defaults( func=rosnode_process )
    rosnode_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rostopic', help='dump rostopic data')
    subparser.set_defaults( func=rostopic_process )
    rostopic_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosservice', help='dump rosservice data')
    subparser.set_defaults( func=rosservice_process )
    rosservice_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rosmsg', help='dump rosmsg data')
    subparser.set_defaults( func=rosmsg_process )
    rosmsg_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('rossrv', help='dump rossrv data')
    subparser.set_defaults( func=rossrv_process )
    rossrv_configure( subparser )

    ## =================================================

    subparser = subparsers.add_parser('roslaunch', help='dump roslaunch data')
    subparser.set_defaults( func=roslaunch_process )
    roslaunch_configure( subparser )

    ## =================================================

    args = parser.parse_args()

    if args.listtools is True:
        tools_list = list( subparsers.choices.keys() )
        print( ", ".join( tools_list ) )
        return

    if "func" not in args:
        ## no command given -- print help message
        parser.print_help()
        sys.exit( 1 )
        return

    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    args.func( args )


if __name__ == '__main__':
    main()
