# Copyright (c) 2022, Arkadiusz Netczuk <dev.arnet@gmail.com>
# All rights reserved.
#
# This source code is licensed under the BSD 3-Clause license found in the
# LICENSE file in the root directory of this source tree.
#

import os
import logging

import argparse

from typing import Dict, Any

from showgraph.io import read_list, read_dict, prepare_filesystem_name
from showgraph.graphviz import Graph, preserve_neighbour_nodes, set_nodes_style

from rosdiagram.utils import get_create_item
from rosdiagram.ros import rostopicdata
from rosdiagram.ros import rosservicedata
from rosdiagram.ros.rosnodedata import get_topics, get_services,\
    get_names_from_list, create_topics_dict, fix_names, split_to_groups,\
    get_services_info, filter_nodes, filter_topics,\
    get_services_from_dict, read_nodes, get_topics_info, filter_ros_nodes_dict,\
    get_topics_dict, get_services_dict
from rosdiagram.graphviztohtml import generate_from_template, set_node_graph_ranks,\
    DEFAULT_ACTIVE_NODE_STYLE, prepare_item_link, set_node_html_attribs, convert_links_list
from rosdiagram.ros.rostopicdata import read_topics, filter_ros_topics_dict
from rosdiagram.ros.rosservicedata import read_services


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


def painter_wrapper( graph: Graph, highlight_list, base_painter=None ):
    if base_painter:
        base_painter( graph )
    if highlight_list:
        style = { "style": "filled",
                  "fillcolor": "yellow"
                  }
        set_nodes_style( graph, highlight_list, style )


## ===================================================================


def generate_graph( nodes_dict, labels_dict=None, show_services=True, full_graph=True, paint_function=None ):
    graph: Graph = None
    if full_graph:
        graph = generate_full_graph( nodes_dict, labels_dict=labels_dict, services_as_labels=show_services )
    else:
        graph = generate_compact_graph( nodes_dict, show_services=show_services, labels_dict=labels_dict )
    if paint_function:
        paint_function( graph )
    return graph


def generate_full_graph( nodes_dict, labels_dict=None, services_as_labels=True, services_as_nodes=False ) -> Graph:
    """Generate graph with services represented as separate graph nodes."""
    if labels_dict is None:
        labels_dict = {}
    dot_graph = Graph()
    dot_graph.setName( "nodes_graph" )
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for node_id, lists in nodes_dict.items():
        node_obj   = dot_graph.addNode( node_id, shape="box" )
        node_label = labels_dict.get( node_id, node_id )
        node_obj.set( "label", node_label )

        topics: list = get_topics( lists )
        for topic_id in topics:
            topic_obj   = dot_graph.addNode( topic_id, shape="ellipse" )
            if topic_obj is None:
                ## already added
                continue
            topic_label = labels_dict.get( topic_id, topic_id )
            topic_obj.set( "label", topic_label )

        if services_as_nodes:
            services: list = get_services( lists )
            for service_id in services:
                service_obj   = dot_graph.addNode( service_id, shape="hexagon" )
                service_label = labels_dict.get( service_id, service_id )
                service_obj.set( "label", service_label )

        if services_as_labels:
            servs_list = lists[ "servs" ]
            node_label = node_label + "\n" + str( len(servs_list) ) + " services"
            node_obj.set( "label", node_label )

    ## add edges
    for node_id, lists in nodes_dict.items():
        pubs_list  = lists[ "pubs" ]
        subs_list  = lists[ "subs" ]
        pubs_list  = get_names_from_list( pubs_list )
        subs_list  = get_names_from_list( subs_list )
        for pub in pubs_list:
            dot_graph.addEdge( node_id, pub )
        for sub in subs_list:
            dot_graph.addEdge( sub, node_id )

        if services_as_nodes:
            servs_list = lists[ "servs" ]
            servs_list = get_names_from_list( servs_list )
            for serv in servs_list:
                dot_graph.addEdge( serv, node_id )

    return dot_graph


def generate_compact_graph( nodes_dict, show_services=True, labels_dict=None ) -> Graph:
    """Generate graph with services represented as number inside node."""
    if labels_dict is None:
        labels_dict = {}
    dot_graph = Graph()
    base_graph = dot_graph.base_graph
    base_graph.set_type( 'digraph' )
    base_graph.set_rankdir( 'LR' )

    ## add nodes
    for node_id, lists in nodes_dict.items():
        node_label = labels_dict.get( node_id, node_id )
        node_obj   = dot_graph.addNode( node_id, shape="box", label=node_label )
        if show_services:
            servs_list = lists[ "servs" ]
            node_label = node_label + "\n" + str( len(servs_list) ) + " services"
            node_obj.set( "label", node_label )

            # for item in servs_list:
            #     dot_graph.addNode( item, shape="hexagon" )
            #     dot_graph.addEdge( node, item )

    topics_dict = create_topics_dict( nodes_dict )

    ## create connections dict
    connections_dict: Dict[ Any, Any ] = {}
    for _, lists in topics_dict.items():
        pubs_list = lists[ "pubs" ]
        subs_list = lists[ "subs" ]
        for pub in pubs_list:
            pub_dict = get_create_item( connections_dict, pub, {} )
            for sub in subs_list:
                sub_counter = get_create_item( pub_dict, sub, 0 )
                #edge = dot_graph.addEdge( pub, sub )
                pub_dict[ sub ] = sub_counter + 1

    ## add edges
    for pub, subs_dict in connections_dict.items():
        for sub, counter in subs_dict.items():
            #edge = dot_graph.addEdge( pub, sub )
            edge = dot_graph.addEdge( pub, sub )
            # if counter < 2:
            #     continue
            edge.set( "label", str(counter) )
    return dot_graph


## =====================================================


def read_nodes_data(nodes_dump_dir, include_ros_internals: bool = False):
    nodes_dict = read_nodes( nodes_dump_dir )
    if not include_ros_internals:
        removed_items = filter_ros_nodes_dict( nodes_dict )
        _LOGGER.info("ignored ROS elements: %s", " ".join(removed_items))
    labels_dict = fix_names( nodes_dict )
    # info_dict  = get_node_info_dict( nodes_dict, labels_dict, msgs_dump_dir, srvs_dump_dir )
    return (nodes_dict, labels_dict)


def generate_graph_data( nodes_dict,
                         node_label_dict,
                         mainfullgraph: bool = False,
                         highlight_list_file=None,
                         paint_function=None,
                         output_dot_file=None,
                         output_png_file=None ):

    highlight_list = read_list( highlight_list_file )

    if not output_dot_file and not output_png_file:
        return

    _LOGGER.info( "generating main graph" )
    graph: Graph = generate_graph( nodes_dict, labels_dict=node_label_dict, full_graph=mainfullgraph,
                                   paint_function=lambda graph: painter_wrapper( graph, highlight_list,
                                                                                 paint_function )
                                   )

    if output_dot_file:
        graph.writeRAW( output_dot_file )
    if output_png_file:
        graph.writePNG( output_png_file )


def generate_node_pages( nodes_output_dir,                                      # pylint: disable=R0913
                         nodes_dict,
                         node_label_dict,
                         nodes_classify_dict,
                         topics_dump_dir,
                         msgs_dump_dir,
                         services_dump_dir,
                         srvs_dump_dir,
                         description_dict,
                         mainfullgraph: bool = False,
                         highlight_list_file=None,
                         paint_function=None):

    highlight_list = None
    if highlight_list_file:
        highlight_list = read_list( highlight_list_file )

    _LOGGER.info( "generating HTML graph" )
    os.makedirs( nodes_output_dir, exist_ok=True )

    generate_pages( nodes_dict, nodes_output_dir,
                    nodes_labels=node_label_dict,
                    nodes_description=description_dict,
                    topics_dump_dir=topics_dump_dir,
                    msgs_dump_dir=msgs_dump_dir,
                    services_dump_dir=services_dump_dir,
                    srvs_dump_dir=srvs_dump_dir,
                    nodes_classify_dict=nodes_classify_dict,
                    paint_function=lambda graph: painter_wrapper( graph, highlight_list, paint_function ),
                    main_full_graph=mainfullgraph
                    )


def generate_pages( nodes_dict, out_dir, nodes_labels=None, nodes_description=None,             # pylint: disable=R0913
                    topics_dump_dir=None, msgs_dump_dir=None, services_dump_dir=None, srvs_dump_dir=None,
                    nodes_classify_dict=None,
                    paint_function=None, main_full_graph=False
                    ):
    if nodes_labels is None:
        nodes_labels = fix_names( nodes_dict )
    if nodes_description is None:
        nodes_description = {}

    OUTPUT_NODES_REL_DIR = os.path.join( "nodes" )
    main_graph_name = "full_graph"

    _LOGGER.info( "generating main graph" )
    ## generate main page graph
    main_graph: Graph = generate_graph( nodes_dict, labels_dict=nodes_labels,
                                        full_graph=main_full_graph, paint_function=paint_function )
    main_graph.setName( main_graph_name )
    set_node_html_attribs( main_graph, OUTPUT_NODES_REL_DIR )

    ## generate sup pages
    sub_output_dir = os.path.join( out_dir, OUTPUT_NODES_REL_DIR )
    os.makedirs( sub_output_dir, exist_ok=True )

    item_filename  = prepare_filesystem_name( main_graph_name )
    main_page_link = os.path.join( os.pardir, item_filename + ".html" )

    _LOGGER.info( "generating subpages" )
    subpages_dict = generate_subpages( sub_output_dir, nodes_dict,
                                       topics_dump_dir, msgs_dump_dir, services_dump_dir, srvs_dump_dir,
                                       nodes_classify_dict,
                                       nodes_labels, nodes_description, main_page_link, paint_function=paint_function )

    nodes_desc    = nodes_description.get( "node", None )
    topics_desc   = nodes_description.get( "topic", None )
    services_desc = nodes_description.get( "service", None )

    ## generate main page
    all_nodes, all_topics, all_services = split_to_groups( nodes_dict )     # get lists of identifiers

    nodes_data_list    = convert_nodes_links_list( all_nodes, subpages_dict, OUTPUT_NODES_REL_DIR,
                                                   nodes_labels, nodes_description=nodes_desc )
    topics_data_list   = convert_links_list( all_topics, subpages_dict, OUTPUT_NODES_REL_DIR,
                                             nodes_labels, nodes_description=topics_desc )
    services_data_list = convert_links_list( all_services, subpages_dict, OUTPUT_NODES_REL_DIR,
                                             nodes_labels, nodes_description=services_desc )

    _LOGGER.info( "generating main page" )
    main_dict = {   "style": {},
                    "graph": main_graph,
                    "graph_label": nodes_labels.get( main_graph_name, main_graph_name ),
                    "nodes_list": nodes_data_list,
                    "topics_list": topics_data_list,
                    "services_list": services_data_list
                    }
    template = "rosnodegraph/nodegraph_main.html"
    generate_from_template( out_dir, main_dict, template_name=template )


## returns dict: { <item_id>: <item_data_dict> }
def generate_subpages( sub_output_dir, nodes_dict,                                        # pylint: disable=R0913,R0914
                       topics_dump_dir, msgs_dump_dir, services_dump_dir, srvs_dump_dir,
                       nodes_classify_dict,
                       nodes_labels, nodes_description, main_page_link, paint_function=None ):
    topics_dict = read_topics( topics_dump_dir )
    if topics_dict is None:
        topics_dict = get_topics_dict( nodes_dict, nodes_labels )

    filter_ros_topics_dict( topics_dict )

    services_dict = read_services( services_dump_dir )
    if services_dict is None:
        services_dict = get_services_dict( nodes_dict, nodes_labels )

    _LOGGER.info( "generating item dicts" )
    all_nodes, all_topics, all_services = split_to_groups( nodes_dict )

    nodes_subpages_dict    = get_items_dict( nodes_dict, all_nodes, nodes_labels, 1,
                                             paint_function=paint_function )
    topics_subpages_dict   = get_items_dict( nodes_dict, all_topics, nodes_labels, 0,
                                             paint_function=paint_function )
    services_subpages_dict = get_items_dict( nodes_dict, all_services, nodes_labels, 0,
                                             paint_function=paint_function )

    _LOGGER.info( "generating item subpages" )
    for _, node_data in nodes_subpages_dict.items():
        node_data[ "template_name" ] = "rosnodegraph/nodegraph_node.html"

    topics_subpages_dict   = get_topic_subpages( topics_subpages_dict, nodes_dict, topics_dict, msgs_dump_dir )
    services_subpages_dict = get_service_subpages( services_subpages_dict, nodes_dict,
                                                   nodes_labels, services_dict, srvs_dump_dir )

    subpages_dict = {}
    subpages_dict.update( nodes_subpages_dict )
    subpages_dict.update( topics_subpages_dict )
    subpages_dict.update( services_subpages_dict )

    nodes_desc    = nodes_description.get( "node", None )
    topics_desc   = nodes_description.get( "topic", None )
    services_desc = nodes_description.get( "service", None )

    ## transform classification data
    pkgs_classify_dict = {}
    if nodes_classify_dict:
        _LOGGER.info( "classifying nodes to packages" )
        for pkg_id, pkg_data in nodes_classify_dict.items():
            pkg_path = pkg_data.get( "path", "" )
            pkg_nodes = pkg_data.get( "nodes", [] )
            for node_id in pkg_nodes:
                classify_data = pkgs_classify_dict.setdefault( f"n_{node_id}", {} )
                classify_data[ "package" ] = pkg_id
                classify_data[ "path" ] = pkg_path

    for item_id, item_dict in subpages_dict.items():
        item_graph = item_dict.get( "graph" )
        if item_graph:
            set_node_graph_ranks( item_graph, item_id )
            set_nodes_style( item_graph, [item_id], style_dict=DEFAULT_ACTIVE_NODE_STYLE )
            set_node_html_attribs( item_graph, "" )

            sub_graph_name = item_graph.getName()
            item_dict[ "graph_label" ] = nodes_labels.get( sub_graph_name, sub_graph_name )

        item_dict[ "main_page_link" ] = main_page_link

        nodes_list                   = item_dict.get( "nodes_list", [] )
        item_dict[ "nodes_list" ]    = convert_links_list( nodes_list, subpages_dict, "",
                                                           nodes_labels, nodes_description=nodes_desc )
        topics_list                  = item_dict.get( "topics_list", [] )
        item_dict[ "topics_list" ]   = convert_links_list( topics_list, subpages_dict, "",
                                                           nodes_labels, nodes_description=topics_desc )
        services_list                = item_dict.get( "services_list", [] )
        item_dict[ "services_list" ] = convert_links_list( services_list, subpages_dict, "",
                                                           nodes_labels, nodes_description=services_desc )

        pkg_classify_data = pkgs_classify_dict.get( item_id, {} )
        item_dict[ "pkg_name" ] = pkg_classify_data.get( "package", "" )
        item_dict[ "pkg_path" ] = pkg_classify_data.get( "path", "" )

        _LOGGER.info( "preparing page for item %s", item_id )
        template = item_dict.get( "template_name", None )
        if template:
            generate_from_template( sub_output_dir, item_dict, template_name=template )

    return subpages_dict


def convert_nodes_links_list( items_lists, sub_items_dict, link_subdir, labels_dict=None, nodes_description=None ):
    if labels_dict is None:
        labels_dict = {}
    if nodes_description is None:
        nodes_description = {}
    converted_list = []
    for item_id in items_lists:
        label = labels_dict.get( item_id, item_id )
        description = nodes_description.get( label, None )
        item_data = sub_items_dict.get( item_id, {} )
        is_subpage = True if item_data else False
        item_link = prepare_item_link( item_id, label, is_subpage, link_subdir )
        if item_data:
            pkg_name = item_data.get("pkg_name", "")
            item_link = ( *item_link, pkg_name )
        if description:
            item_link = ( *item_link, description )
        converted_list.append( item_link )
    return converted_list


def get_topic_subpages( topics_subpages_dict, nodes_dict, topics_dict, msgs_dump_dir ):
    topic_labels = rostopicdata.fix_names( topics_dict )
    topics_info  = get_topics_info( nodes_dict, topics_dict, msgs_dump_dir )

    for topic_id, topic_data in topics_info.items():
        pubs_list = topic_data.get( "pubs", [] )
        subs_list = topic_data.get( "subs", [] )

        pubs_names = []
        if pubs_list:
            pubs_names = [ get_label( topic_labels, item_id, "<unknown>" ) for item_id in pubs_list ]
        subs_names = []
        if subs_list:
            subs_names = [ get_label( topic_labels, item_id, "<unknown>" ) for item_id in subs_list ]

        sub_dict = topics_subpages_dict[ topic_id ]
        sub_dict[ "template_name" ] = "rosnodegraph/nodegraph_topic.html"
        sub_dict[ "topic_name" ]    = get_label( topic_labels, topic_id, "<unknown>" )
        sub_dict[ "topic_pubs" ]    = pubs_names
        sub_dict[ "topic_subs" ]    = subs_names
        sub_dict[ "msg_type" ]      = topic_data.get( "type", "" )
        sub_dict[ "msg_content" ]   = topic_data.get( "content", "" )
    return topics_subpages_dict


def get_service_subpages( services_subpages_dict, nodes_dict, nodes_labels, services_dict, srvs_dump_dir ):
    services_labels = rosservicedata.fix_names( services_dict )
    services_info = get_services_info( nodes_dict, services_dict, srvs_dump_dir )

    for service_id, service_data in services_info.items():
        sub_dict = services_subpages_dict[ service_id ]
        sub_dict[ "template_name" ] = "rosnodegraph/nodegraph_service.html"
        sub_dict[ "srv_name" ]      = get_label( services_labels, service_id, "<unknown>" )
        sub_dict[ "msg_type" ]      = service_data.get( "type", "" )
        sub_dict[ "msg_content" ]   = service_data.get( "content", "" )

        listener_id = service_data.get( "listener", "" )
        if listener_id:
            label = nodes_labels.get( listener_id, listener_id )
            is_subpage = listener_id in nodes_dict
            srv_link = prepare_item_link( listener_id, label, is_subpage, "" )
            if srv_link:
                sub_dict[ "srv_listener" ] = srv_link
    return services_subpages_dict


def get_items_dict( nodes_dict, items_list, label_dict, neighbour_range, paint_function=None ):
    sub_items = {}
    for item_id in items_list:
        item_dict = {}
        sub_items[ item_id ] = item_dict
        item_graph: Graph = generate_full_graph( nodes_dict, labels_dict=label_dict )
        preserve_neighbour_nodes( item_graph, [item_id], neighbour_range )
        if paint_function:
            paint_function( item_graph )

        item_dict[ "graph" ]       = item_graph
        item_dict[ "msg_name" ]    = ""
        item_dict[ "msg_type" ]    = ""
        item_dict[ "msg_content" ] = ""

        ## get lists of node pubs, subs and servs
        graph_names   = list( item_graph.getNodeNamesAll() )
        nodes_list    = sorted( filter_nodes( nodes_dict, graph_names ) )
        topics_list   = sorted( filter_topics( nodes_dict, graph_names ) )
        services_list = sorted( get_services_from_dict( nodes_dict, [ item_id ] ) )

        item_dict[ "nodes_list" ]    = nodes_list
        item_dict[ "topics_list" ]   = topics_list
        item_dict[ "services_list" ] = services_list

    return sub_items


def get_label( label_dict, item_id, default_name="<unknown>" ):
    item_name = label_dict.get( item_id, None )
    if item_name:
        return item_name
    return default_name


## ===================================================================


def configure_parser( parser ):
    parser.description = 'rosnode connection graph'
    parser.add_argument( '-la', '--logall', action='store_true', help='Log all messages' )
    # pylint: disable=C0301
    parser.add_argument( '--nodesdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosnode' output" )
    parser.add_argument( '--topicsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rostopic' output" )
    parser.add_argument( '--msgsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosmsg' output" )
    parser.add_argument( '--servicesdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rosservice' output" )
    parser.add_argument( '--srvsdumppath', action='store', required=False, default="",
                         help="Path to directory containing dumped 'rossrv' output" )
    parser.add_argument( '--classifynodesfile', action='store', required=False, default="",
                         help="Nodes classification input file" )
    parser.add_argument( '--highlightitems', action='store', required=False, default="", help="File with list of items to highlight" )
    parser.add_argument( '--descriptionjson', action='store', required=False, default="", help="Path to JSON file with items description" )
    parser.add_argument( '-mfg', '--mainfullgraph', action='store_true', help="Generate main full graph instead of compact one" )
    parser.add_argument( '-iri', '--includerosinternals', action='store_true', help="Include ROS internal items like /rosout and /record_*" )
    parser.add_argument( '--outraw', action='store', required=False, default="", help="Graph RAW output" )
    parser.add_argument( '--outpng', action='store', required=False, default="", help="Graph PNG output" )
    parser.add_argument( '--outhtml', action='store_true', help="Output HTML" )
    parser.add_argument( '--outdir', action='store', required=False, default="", help="Output HTML" )


def process_arguments( args, paint_function=None ):
    logging.basicConfig()
    if args.logall is True:
        logging.getLogger().setLevel( logging.DEBUG )
    else:
        logging.getLogger().setLevel( logging.INFO )

    nodes_output_dir = args.outdir
    topics_dump_dir = args.topicsdumppath
    msgs_dump_dir = args.msgsdumppath
    services_dump_dir = args.servicesdumppath
    srvs_dump_dir = args.srvsdumppath
    nodes_classify_file = args.classifynodesfile

    nodes_dict, labels_dict = read_nodes_data(args.nodesdumppath, args.includerosinternals)

    if args.outraw or args.outpng:
        generate_graph_data( nodes_dict, labels_dict,
                             mainfullgraph=args.mainfullgraph,
                             highlight_list_file=args.highlightitems,
                             paint_function=paint_function,
                             output_dot_file=args.outraw,
                             output_png_file=args.outpng
                             )

    ##
    ## generate HTML data
    ##
    if args.outhtml and nodes_output_dir:
        nodes_classify_dict = read_dict( nodes_classify_file )
        description_dict = read_dict( args.descriptionjson )

        generate_node_pages( nodes_output_dir,
                             nodes_dict,
                             labels_dict,
                             nodes_classify_dict=nodes_classify_dict,
                             topics_dump_dir=topics_dump_dir,
                             msgs_dump_dir=msgs_dump_dir,
                             services_dump_dir=services_dump_dir,
                             srvs_dump_dir=srvs_dump_dir,
                             description_dict=description_dict,
                             mainfullgraph=args.mainfullgraph,
                             highlight_list_file=args.highlightitems,
                             paint_function=paint_function )


def main():
    parser = argparse.ArgumentParser()
    configure_parser( parser )
    args = parser.parse_args()
    process_arguments( args )
