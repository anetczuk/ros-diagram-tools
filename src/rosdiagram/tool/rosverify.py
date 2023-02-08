#!/usr/bin/env python3
# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import logging
import argparse

import subprocess


_LOGGER = logging.getLogger(__name__)


def get_worspaces( input_workspace="" ):
    ws_arg = ""
    if input_workspace:
        ws_arg = f"--workspace {input_workspace}"

    ## output, error =
    output, _ = execute_command( f"catkin build {ws_arg} --dry-run" )
    workspaces = []
    output_lines = output.decode("utf-8")
    for line in output_lines.splitlines():
        if line.startswith("Extending:"):
            data_start = line.find("/")
            raw_data = line[data_start:]
            extended_ws = raw_data.split(":")
            workspaces.extend( extended_ws )
        if line.startswith("Workspace:"):
            data_start = line.find("/")
            raw_data = line[data_start:]
            workspaces.insert( 0, raw_data )
    return workspaces


def get_packages( workspace_path ):
    ## output, error =
    output, _ = execute_command( f"catkin list -u --directory {workspace_path}" )
    output_lines = output.decode("utf-8")
    return output_lines.splitlines()


def execute_command( command_string ):
    with subprocess.Popen( command_string.split(), stdout=subprocess.PIPE ) as process:
        return process.communicate()


def find_overlays( workspaces_list ):
    ret_data = []

    all_packages = []

    for ws_path in workspaces_list:
        ws_packages = get_packages( ws_path )
        all_packages.append( ws_packages )
    # print( "found packages:", all_packages )

    ws_num = len( workspaces_list )
    for i in range(0, ws_num - 1):
        ## curr_ws   = workspaces_list[i]
        curr_pkgs = all_packages[i]
        #rest_pkgs = all_packages[ i + 1: ]
        for pkg in curr_pkgs:
            for r in range(i + 1, ws_num):
                rest_pkgs = all_packages[ r ]
                if pkg in rest_pkgs:
                    ## rest_ws = workspaces_list[ r ]
                    ret_data.append( [pkg, i, r ] )
    return ret_data


## =======================================================================


def main():
    parser = argparse.ArgumentParser(description='package overlay detector')
    parser.add_argument( '-w', '--workspace', action='store', required=False, default="",
                         help="Workspace directory to analyze" )

    args = parser.parse_args()

    logging.basicConfig()
    logging.getLogger().setLevel( logging.INFO )

    input_workspace = args.workspace
    if not input_workspace:
        input_workspace = ""

    workspaces = get_worspaces( input_workspace )
    _LOGGER.info( "found workspaces: %s", workspaces )

    overlay_data = find_overlays( workspaces )

    all_overlays  = []
    main_overlays = []
    for overlay in overlay_data:
        pkg        = overlay[0]
        curr_index = overlay[1]
        curr_ws    = workspaces[ curr_index ]
        rest_ws    = workspaces[ overlay[2] ]
        _LOGGER.info( "package '%s' in '%s' overlays package in '%s'", pkg, curr_ws, rest_ws )
        all_overlays.append( pkg )
        if curr_index == 0:
            main_overlays.append( pkg )

    _LOGGER.info( "all overlay packages: %s", all_overlays )
    _LOGGER.info( "workspace overlay packages: %s", main_overlays )
