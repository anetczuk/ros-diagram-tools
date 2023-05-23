# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging
import argparse
import re
import hashlib
import pprint
from typing import List, Dict, Any

from dataclasses import dataclass

from PIL import Image, ImageDraw

from showgraph.io import read_file, write_file

from rosdiagram import texttemplate
from rosdiagram.textutils import time_to_string


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


@dataclass
class Job():
    name: str         = None
    start_time: float = None
    end_time: float   = None

    def duration(self):
        return self.end_time - self.start_time


class Schedule():

    def __init__(self, jobs_list=None, high_load_duration=0.0, start_gap_duration=0.0):
        self.queues: List[ list ] = []
        if jobs_list is not None:
            self.parseList( jobs_list )
        self.high_load_duration = high_load_duration
        self.start_gap_duration = start_gap_duration

    def parseList( self, jobs_list: List[Job] ):
        self.queues.clear()
        for job in jobs_list:
            self._addJob( job )

    def queuesNum(self):
        return len( self.queues )

    def jobsList(self) -> List[Job]:
        ret_list = []
        for queue in self.queues:
            ret_list.extend( queue )
        return ret_list

    def endTime(self):
        total_end_time = 0
        for queue in self.queues:
            if not queue:
                continue
            last_job: Job = queue[-1]
            last_end = last_job.end_time
            total_end_time = max( total_end_time, last_end )
        return total_end_time

    def getQueuesStats(self):
        q_size = len( self.queues )
        if q_size < 1:
            return []
        pipeline_duration = self.endTime()
        jobs_duration = 0.0
        queue_time_list = []
        for q in range(0, q_size):
            queue = self.queues[q]
            name = f"thread {q}"
            if not queue:
                queue_time_list.append( (name, 0.0, 0.0) )
                continue
            queue_time = 0.0
            for job in queue:
                queue_time += job.duration()
            queue_time_list.append( ( name, queue_time, queue_time / pipeline_duration ) )
            jobs_duration += queue_time

        queue_time_list.append( ( "OVERALL", jobs_duration, jobs_duration / pipeline_duration / q_size ) )
        return queue_time_list

    def getCriticalPath( self ) -> List[Job]:
        ret_list = []
        infinity = float("inf")
        last_job: Job = self.getBefore( infinity )
        if last_job is None:
            return ([], 0.0)
        ret_gap = 0.0
        while True:
            ret_list.append( last_job )
            start_time = last_job.start_time
            prev_job = self.getBefore( start_time )
            if prev_job is None:
                break
            ret_gap += last_job.start_time - prev_job.end_time
            last_job = prev_job
        ret_list.reverse()
        return (ret_list, ret_gap)

    def getBefore( self, end_time, ref_job=None ) -> Job:
        last_job = None
        for queue in self.queues:
            for job in queue:
                if job == ref_job:
                    continue
                job_end_time = job.end_time
                if job_end_time > end_time:
                    continue
                if not last_job:
                    last_job = job
                    continue
                if job_end_time > last_job.end_time:
                    last_job = job
        return last_job

    def _addJob( self, job: Job ):
        curr_start_time = job.start_time
        for queue in self.queues:
            if not queue:
                continue
            last_job = queue[ -1 ]
            if curr_start_time >= last_job.end_time:
                queue.append( job )
                return
        ## add next processor queue
        self.queues.append( [job] )


## ===============================================================


def generate_pages( schedule: Schedule, out_dir, config_params_dict=None, scale_sec_step=20, scale_px_num=100 ):
    if config_params_dict is None:
        config_params_dict = {}

    # draw_path = os.path.join( out_dir, 'schedule.png' )
    # draw_shedule( schedule, draw_path )

    graph_path = os.path.join( out_dir, 'schedule.puml' )
    generate_plant_graph( schedule, graph_path, scale_sec_step, scale_px_num )
    # generate_dot_graph( schedule )

    params_dict: Dict[ str, Any ] = {}
    generate_graph_page( schedule, params_dict, out_dir )


def generate_graph_page( schedule: Schedule, item_config_dict, output_dir ):
    page_params = item_config_dict.copy()

    build_time = schedule.endTime()
    total_time = time_to_string( build_time )

    critical_path, crit_gap = schedule.getCriticalPath()
    critical_path = print_critical_path( schedule, critical_path )

    packages_sum_time = 0.0
    packages_list = schedule.jobsList()
    for job in packages_list:
        packages_sum_time += job.duration()
    packages_total_time = time_to_string( packages_sum_time )

    dur_list      = print_durations( packages_list, True )

    packages_list = print_durations( packages_list, False )
    packages_list.sort()

    pipeline_times = schedule.getQueuesStats()

    ## prepare input for template
    page_params.update( {   "body_color":   "#bbbbbb",
                            "svg_name":     "schedule.svg",
                            # "main_page_link":   main_page_link,

                            "total_time": total_time,
                            "packages_total_time": packages_total_time,
                            "high_load_duration": time_to_string( schedule.high_load_duration ),
                            "start_gap_duration": time_to_string( schedule.start_gap_duration ),
                            "critical_gap_duration": time_to_string( crit_gap ),
                            
                            "pipeline_times": pipeline_times,
                            
                            "critical_path": critical_path,
                            "duration_list": dur_list,
                            "packages_list": packages_list
                            } )

    template_path = os.path.join( SCRIPT_DIR, os.pardir, "template", "build_time_page.html.tmpl" )
    main_out_path = os.path.join( output_dir, "full_graph.html" )

    texttemplate.generate( template_path, main_out_path, INPUT_DICT=page_params )

    _LOGGER.info( "writing main page: file://%s", main_out_path )


## ===================================================================


def read_build_log( log_path ) -> List[Job]:
    content = read_file( log_path )
    if content is None:
        _LOGGER.warning( "unable to read content from file '%s'", log_path )
        return

    order_list = []
    start_dict = {}
    end_dict   = {}

    ## sometimes catkin not starts next job exactly after one finishes, but waits some time
    recent_time        = 0.0
    high_load_duration = 0.0
    last_finish_time   = 0.0
    start_gap_duration = 0.0

    for line in content.splitlines():
        line = line.strip()

        build_time = get_build_timestamp( line )
        if build_time:
            if "High Load" in line:
                high_duration = build_time - recent_time
                high_load_duration += high_duration
            recent_time = max( recent_time, build_time )
            continue

        line_log = get_after( line, "Starting >>> " )
        if line_log:
            gap_duration = recent_time - last_finish_time
            start_gap_duration += gap_duration
            package_name = line_log
            start_dict[ package_name ] = recent_time
            order_list.append( package_name )
            print( "Starting >", package_name, "<", recent_time )
            continue

        line_log = get_after( line, "Finished <<< " )
        if line_log:
            last_finish_time = recent_time
            split_content = re.split( r"\[|]", line_log )
            package_name = split_content[0]
            package_name = package_name.strip()

#             time_line = split_content[1]
#             time_line = time_line.strip()
# 
#             numbers = re.findall( r"(\d*\.*\d+)", time_line )
#             total_seconds = 0.0
#             if len(numbers) == 1:
#                 total_seconds += float( numbers[0] )
#             elif len(numbers) == 2:
#                 total_seconds += float( numbers[0] ) * 60
#                 total_seconds += float( numbers[1] )
#             else:
#                 raise ValueError( f"unhandled case: {time_line}" )
#
#             start_time = start_dict[ package_name ]
#             last_end_time = start_time + total_seconds
#             recent_time = max( recent_time, last_end_time )

            end_dict[ package_name ] = recent_time
            ## print( "Finishing >", package_name, "<", start_time, recent_time )
            continue

    jobs_list = []
    for package_name in order_list:
#         if event[1] is not "start":
#             continue
#         package_name = event[0]
        start_time = start_dict[ package_name ]
        end_time   = end_dict[ package_name ]
        item = Job( package_name, start_time, end_time )
        ## print( "adding >", package_name, "<" )
        jobs_list.append( item )
        #print( ">", item, "<" )

    print("jobs list:")
    pprint.pprint( jobs_list )

    schedule = Schedule( jobs_list, high_load_duration, start_gap_duration )
    return schedule


def get_after( content, start ):
    try:
        pos = content.index( start )
        return content[ pos + len(start): ]
    except ValueError:
        pass
    return None


def get_build_timestamp( content ):
    ## [build 36.6 s]
    times_list = re.findall( r"\[build (\S+) s\]", content )
    if len(times_list) != 1:
        return None

    time_text = times_list[0]
    numbers = re.findall( r"(\d*\.*\d+)", time_text )

    total_seconds = 0.0
    if len(numbers) == 1:
        total_seconds += float( numbers[0] )
    elif len(numbers) == 2:
        total_seconds += float( numbers[0] ) * 60
        total_seconds += float( numbers[1] )
    else:
        raise ValueError( f"unhandled case: {content}" )

    return total_seconds


## ===================================================================


def draw_shedule( schedule: Schedule, out_path ):
    queues_num = schedule.queuesNum()
    total_end_time = schedule.endTime()
    total_end_time = int( total_end_time )

    pprint.pprint( schedule )
    print( "queues num:", queues_num )
    print( "total time:", total_end_time )

    rect_height = 20
    margin      = 10

    ## init canvas
    canvas = ( total_end_time + 2 * margin,
               queues_num * ( margin + rect_height ) + margin
               )
    im = Image.new('RGBA', canvas, (255, 255, 255, 255))
    draw = ImageDraw.Draw( im )

    for q in range(0, queues_num):
        queue = schedule.queues[ q ]
        queue_x = margin
        queue_y = q * ( margin + rect_height ) + margin
        for job in queue:
            left_x   = int( queue_x + job.start_time )
            right_x  = int( queue_x + job.end_time )
            top_y    = int( queue_y )
            bottom_y = int( queue_y + rect_height )
            rect_coords = [ left_x, top_y, right_x, bottom_y ]

            job_name  = job.name
            item_hash = hashlib.sha256( job_name.encode('utf-8') ).hexdigest()
            color_int = int( item_hash, 16 )
            color     = rgb_from_int( color_int )
            draw.rectangle( rect_coords, fill=color, outline=(0, 0, 0, 255) )

    im.save( out_path )


def rgb_from_int( color_num ):
    color_int = color_num % (256 * 256 * 256)
    blue  = color_int & 255
    green = (color_int >> 8) & 255
    red   = (color_int >> 16) & 255
    return (red, green, blue)


## ===================================================================


def print_durations( jobs_list: List[Job], sort_list=False ):
    items = []
    for job in jobs_list:
        duration = job.duration()
        items.append( (job, duration ) )

    if sort_list:
        items.sort( key=lambda item: item[1], reverse=True )

    ret_list = []
    for item in items:
        job        = item[0]
        duration   = time_to_string( item[1] )
        start_time = time_to_string( job.start_time )
        end_time   = time_to_string( job.end_time )
        ret_list.append( ( job.name, duration, start_time, end_time ) )
    return ret_list


def print_critical_path( schedule: Schedule, jobs_list: List[Job], sort_list=False ):
    items = []
    for job in jobs_list:
        duration = job.duration()
        gap_time = 0.0
        prev_job = schedule.getBefore( job.end_time, job )
        if prev_job:
            gap_time = job.end_time - prev_job.end_time
        else:
            gap_time = job.end_time
        items.append( (job, duration, gap_time ) )

    if sort_list:
        items.sort( key=lambda item: item[1], reverse=True )

    ret_list = []
    for item in items:
        job        = item[0]
        duration   = time_to_string( item[1] )
        gap        = time_to_string( item[2] )
        start_time = time_to_string( job.start_time )
        end_time   = time_to_string( job.end_time )
        ret_list.append( ( job.name, duration, gap, start_time, end_time ) )
    return ret_list


def generate_plant_graph( schedule: Schedule, out_path, scale_sec_step=20, scale_px_num=100 ):
    content = f"""\
@startuml

'comment

scale {scale_sec_step} as {scale_px_num} pixels

"""

    critical_path, _ = schedule.getCriticalPath()

    queues_num = len( schedule.queues )
    for q in range(0, queues_num):
        content += f"concise \"thread {q}\" as queue{q}\n"

    content += "\n"

    for q in range(0, queues_num):
        content += f"@queue{q}\n"
        queue = schedule.queues[ q ]
        queue_size = len(queue)
        if queue_size < 1:
            ## nothing to add
            continue
        if queue_size < 2:
            ## only one element
            curr_job = queue[0]
            color = ""
            if curr_job in critical_path:
                color = "#red"
            curr_start_time = int( curr_job.start_time )
            content += f"{curr_start_time} is \"{curr_job.name}\" {color}\n"
            curr_end_time = int( curr_job.end_time )
            content += f"{curr_end_time} is {{hidden}}\n"
            continue

        ## normal case
        for j in range( 0, queue_size ):
            curr_job = queue[j]
            color = ""
            if curr_job in critical_path:
                color = "#red"
            curr_start_time = int( curr_job.start_time )
            content += f"{curr_start_time} is \"{curr_job.name}\" {color}\n"
            curr_end_time = int( curr_job.end_time )
            if j != queue_size - 1:
                next_job = queue[j + 1]
                next_start_time = int( next_job.start_time )
                if next_start_time > curr_end_time:
                    content += f"{curr_end_time} is {{hidden}}\n"
            else:
                ## last element -- add hidden
                content += f"{curr_end_time} is {{hidden}}\n"

        content += "\n"

    content += "@enduml\n"

    write_file( out_path, content )


# def generate_dot_graph( schedule: Schedule ):
#     graph: Graph = Graph()
#     base_graph = graph.base_graph
#     base_graph.set_type( 'digraph' )
#     base_graph.set_rankdir( 'LR' )
#
#     for queue in schedule:
#         for job in queue:
#             node = graph.addNode( job[0], shape="box" )
#             duration = int( (job[2] - job[1]) / 2 )
#             node.set( "width", duration )
#             node.set( "fixedsize", "true" )
#
#         ## add edges between jobs in queue
#         queue_size = len(queue)
#         if queue_size > 1:
#             for j in range( 0, queue_size - 1 ):
#                 curr_job = queue[j]
#                 next_job = queue[j + 1]
# #                 graph.addEdge( curr_job[0], next_job[0] )
#                 edge = graph.addEdge( curr_job[0], next_job[0] )
#                 if edge is not None:
#                     edge.set( "weight", 9999 )
#
#     ## add connections between queues
#     for queue in schedule:
#         for job in queue:
#             end_time = job[2]
#             for other_queue in schedule:
#                 if other_queue is queue:
#                     continue
#                 for other_job in other_queue:
#                     start_time = other_job[1]
#                     if start_time == end_time:
#                         graph.addEdge( job[0], other_job[0] )
# #                         edge = graph.addEdge( job[0], other_job[0] )
# #                         if edge is not None:
# #                             edge.set( "weight", 9999 )
#
#     graph.writeRAW( "schedule.gv.txt" )
#     graph.writePNG( "schedule.gv.png" )
#     graph.write( "schedule.gv.svg", file_format='svg')


## ===================================================================


def configure_parser( parser ):
    parser.description = 'catkin build schedule'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '-f', '--file', action='store', required=False, default="",
                         help="Read catkin build log file" )
    parser.add_argument( '-st', '--scalesecstep', action='store', required=False, default=20, help="Scale time step. Timeline will be divided into steps of given size." )
    parser.add_argument( '-sp', '--scalepxnum', action='store', required=False, default=100, help="Scale pixel number - width in pixels of each timeline step." )
    parser.add_argument( '--outhtml', action='store_true', help="Output HTML" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    schedule = read_build_log( args.file )
    if schedule is None:
        return

    ##
    ## generate HTML data
    ##
    if args.outhtml and len( args.outdir ) > 0:
        _LOGGER.info( "generating HTML graph" )
        scale_sec_step = args.scalesecstep
        scale_px_num   = args.scalepxnum
        generate_pages( schedule, args.outdir, scale_sec_step=scale_sec_step, scale_px_num=scale_px_num )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
