#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2022 Arkadiusz Netczuk <dev.arnet@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import os
import logging

from rosdiagram.io import read_file, prepare_filesystem_name
from rosdiagram.graphviz import Graph, get_nodes_names, preserve_neighbour_nodes,\
    unquote_name, set_nodes_style, unquote_name_list, set_node_labels,\
    get_node_label
from rosdiagram import texttemplate


_LOGGER = logging.getLogger(__name__)

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )


## ===================================================================


LABEL_DICT_KEY       = "label_dict"
NEIGHBOURS_RANGE_KEY = "neighbours_range"


def generate_graph_html( output_dir, params_dict=None ):
    if params_dict is None:
        params_dict = {}
    generator = HtmlGenerator( params_dict )
    generator.output_dir = output_dir
    generator.generate()


##
class BaseHtmlGenerator():

    def __init__(self, graph=None, output_dir=None, params_dict=None):
        if isinstance( params_dict, ParamsDict ) is False:
            params_dict   = ParamsDict( params=params_dict )
        self.params       = params_dict
        self.graph: Graph = graph
        self.output_dir   = output_dir
        self._node_group_dict = None

    def getNodeGroupDict(self):
        if self._node_group_dict is not None:
            return self._node_group_dict
        groups_dict = {}
        def_neighbours_range = self.params.get( NEIGHBOURS_RANGE_KEY, 0 )
        groups_list          = self.params.get( "groups", [] )
        for grp in groups_list:
            items_list = grp.get( "items", [] )
            n_range    = grp.get( NEIGHBOURS_RANGE_KEY, def_neighbours_range )
            for itm in items_list:
                groups_dict[ itm ] = { NEIGHBOURS_RANGE_KEY: n_range }
        self._node_group_dict = groups_dict
        return self._node_group_dict

    def splitNodesToGroups( self, names_list ):
        groups_list = self.params.get( "groups", [] )
        if len( groups_list ) < 1:
            return None
        ret_groups = []
        for grp in groups_list:
            items = grp.get( "items", [] )
            grp_dict  = {}
            grp_dict[ "title" ] = grp.get("title", "")
            grp_items = []
            for node_name in names_list:
                if node_name in items:
                    grp_items.append( node_name )
            grp_dict[ "items" ] = grp_items
            ret_groups.append( grp_dict )
        return ret_groups

    def storeDataForHtml( self ):
        graph_name    = self.graph.getName()
        item_filename = prepare_filesystem_name( graph_name )

#         data_out = os.path.join( self.output_dir, item_filename + ".gv.txt" )
#         self.graph.writeRAW( data_out )
        data_out = os.path.join( self.output_dir, item_filename + ".png" )
        self.graph.writePNG( data_out )
        data_out = os.path.join( self.output_dir, item_filename + ".map" )
        self.graph.writeMap( data_out )


##
class HtmlGenerator( BaseHtmlGenerator ):

    OUTPUT_NODES_REL_DIR = os.path.join( "nodes" )

    def __init__( self, params_dict=None ):
        super().__init__( params_dict=params_dict )
        self.output_nodes_dir      = None

    def generate( self ):
        self.generateMain()
        self.generateNodes()

    def generateMain( self, graph_name: str = None ):
        self._setMainGraph( graph_name )

        main_engine = self.params.get( "main_engine" )
        if main_engine is not None:
            self.graph.setEngine( main_engine )

        label_dict = self.params.get( LABEL_DICT_KEY, None )
        set_node_labels( self.graph, label_dict, override=False )

        set_node_html_attribs( self.graph, self.OUTPUT_NODES_REL_DIR )
        self.prepareMainPage()

    ## generate and store neighbour graphs
    def generateNodes( self ):
        self._setMainGraph()

        self.output_nodes_dir = os.path.join( self.output_dir, self.OUTPUT_NODES_REL_DIR )
        os.makedirs( self.output_nodes_dir, exist_ok=True )

        full_graph_name = self.graph.getName()
        main_page_path  = os.path.join( os.pardir, full_graph_name + ".html" )
        back_link = f"""\
<a href="{main_page_path}">back to Main graph</a>
<br />"""

        def_neighbours_range  = self.params.get( NEIGHBOURS_RANGE_KEY, 0 )
        active_node_style     = self.params.getValue( "active_node_style", DEFAULT_ACTIVE_NODE_STYLE )
        label_dict            = self.params.get( LABEL_DICT_KEY, None )

        groups_dict = self.getNodeGroupDict()
        all_names   = list( groups_dict.keys() )
        if len(all_names) < 1:
            all_nodes = self.graph.getNodesAll()
            all_names = get_nodes_names( all_nodes )

        engine_map = self._getNodeEngineMap( all_names )

        for item in all_names:
            _LOGGER.info( "preparing page for item %s", item )

            ## generate subgraph
            node_graph: Graph = self._spawnGraph()

            engine = engine_map.get( item, None )
            if engine is not None:
                node_graph.setEngine( engine )

            item_group = groups_dict.get( item, {} )
            n_range    = item_group.get( NEIGHBOURS_RANGE_KEY, def_neighbours_range )
            node_graph.setName( item )
            preserve_neighbour_nodes( node_graph, [item], n_range )

            ### set rank for neighbour nodes
            node_graph.setNodesRankByName( [item], 100 )

            source_layers = node_graph.getSourceNames( item, 0 )
            source_names = source_layers[0]
            source_names = unquote_name_list( source_names )
            node_graph.setNodesRankByName( source_names, 50 )

            destination_layers = node_graph.getDestinationNames( item, 0 )
            destination_names = destination_layers[0]
            destination_names = unquote_name_list( destination_names )
            node_graph.setNodesRankByName( destination_names, 150 )

            set_node_labels( node_graph, label_dict )

            set_nodes_style( node_graph, [item], style_dict=active_node_style )
            set_node_html_attribs( node_graph, "", filter_nodes=all_names )

            self.prepareNodePage( node_graph, back_link )

    def _setMainGraph( self, graph_name: str = None ):
        if self.graph is None:
            self.graph = self.params.get( "main_graph", None )
        if self.graph is None:
            self.graph = self._spawnGraph()

        if graph_name is None:
            self.graph.setName( "full_graph" )
        else:
            self.graph.setName( graph_name )

    def _spawnGraph( self ) -> Graph:
        graph_factory = self.params.get( "graph_factory", None )
        if graph_factory is None:
            raise RuntimeError( "graph_factory not set" )
        return graph_factory()

    def _getNodeEngineMap(self, nodes_list):
        engine_map = {}
        for node in nodes_list:
            node_engine = self.params.getNamed( "node_engine", node )
            if node_engine is None:
                node_engine = self.params.getValue( "main_engine" )
            if node_engine is not None:
                engine_map[ node ] = node_engine
        return engine_map

    ## ==================================================================

    def prepareMainPage( self ):
        _LOGGER.info( "generating main page" )
        generator = GraphHtmlGenerator( self.graph, self.output_dir, params_dict=self.params )
        generator.graph_top_content = "Main graph"
        generator.type_label = "graph"

        full_graph = self._spawnGraph()
        set_node_html_attribs( full_graph, self.OUTPUT_NODES_REL_DIR )
        generator.custom_bottom_content = generator.generateIndexContent( full_graph )

        generator.generate()

#         ## index page
#         graph_name = self.graph.getName()
#         index_out  = os.path.join( self.output_dir, "index.html" )
#         index_html = f"""\
# <body>
#     <a href="{graph_name}.html">big graph</a>
# </body>
# """
#         write_file( index_out, index_html )

    def prepareNodePage( self, node_graph, back_link="" ):
        generator = GraphHtmlGenerator( node_graph, self.output_nodes_dir, params_dict=self.params )
        generator.graph_top_content = back_link
        generator.type_label = "node"

        generator.generate()


##
class GraphHtmlGenerator( BaseHtmlGenerator ):

    def __init__(self, graph=None, output_dir=None, params_dict=None ):
        super().__init__( graph=graph, output_dir=output_dir, params_dict=params_dict )

        self.graph_top_content     = ""
        self.custom_bottom_content = ""
        self.type_label            = ""

    def generate( self ):
        self.storeDataForHtml()

        graph_name     = self.graph.getName()
        graph_filename = prepare_filesystem_name( graph_name )

        map_out    = os.path.join( self.output_dir, graph_filename + ".map" )
        graph_map  = read_file( map_out )
        if graph_map is None:
            _LOGGER.error( "unable to generate html page" )
            return

        body_color     = self.params.get( "body_color", "#bbbbbb" )
        head_css_style = self.params.get( "head_css_style", "" )

        label_dict  = self.params.get( LABEL_DICT_KEY, {} )
        graph_label = label_dict.get( graph_name, graph_name )

        alt_text = graph_label
        if len(self.type_label) > 0:
            alt_text = self.type_label + " " + alt_text

        info_dict = self.params.get( "graph_info_dict", {} )
        info_content = info_dict.get( graph_name, "" )

        if self.custom_bottom_content:
            bottom_content = self.custom_bottom_content
        else:
            bottom_content = self.generateIndexContent( self.graph )

        page_params = { "body_color":       body_color,
                        "head_css_style":   head_css_style,
                        "top_content":      self.graph_top_content,
                        "info_content":     info_content,
                        "bottom_content":   bottom_content,

                        "graph_name":     graph_name,
                        "graph_filename": graph_filename,
                        "alt_text":       alt_text,
                        "graph_map":      graph_map
                        }

        template_path = os.path.join( SCRIPT_DIR, "template", "nodegraph_page.html.tmpl" )
        html_out      = os.path.join( self.output_dir, graph_filename + ".html" )
#         index_html    = GRAPH_PAGE_TEMPLATE.format( **page_params )
#         write_file( html_out, index_html )

        texttemplate.generate( template_path, html_out, INPUT_DICT=page_params )

    def generateIndexContent( self, graph ):
        label_dict = self.params.get( LABEL_DICT_KEY, {} )

        nodes_dict     = graph.getNodeNamesDict()
        all_names_list = list( nodes_dict.keys() )

        nodes_groups = self.splitNodesToGroups( all_names_list )
        if nodes_groups is None:
            all_grp = {}
            all_grp[ 'title' ] = "Graph items"
            all_grp[ 'items' ] = all_names_list
            nodes_groups = [ all_grp ]

        bottom_content = ""
        for grp in nodes_groups:
            grp_names = grp.get( "items", [] )
            grp_size = len( grp_names )
            if grp_size < 1:
                continue
            grp_names.sort()
            grp_title = grp.get("title", "" )
            bottom_content += f"\n{grp_title} ({grp_size}):\n"
            bottom_content += "<ul>\n"
            for itm_name in grp_names:
                node = nodes_dict.get( itm_name, None )
                if node is None:
                    continue
                node_label = label_dict.get( itm_name, None )
                if node_label is None:
                    node_label = get_node_label( node )
                node_url   = node.get( "href" )
                if node_url is not None:
                    bottom_content += f"""<li><a href="{node_url}"><code>{node_label}</code></a></li>\n"""
            bottom_content += "</ul><br />\n"

        return bottom_content


## =============================================================


DEFAULT_ACTIVE_NODE_STYLE = { "style": "filled",
                              "fillcolor": "brown1"
                              }


GRAPH_PAGE_TEMPLATE = """\
<html>
<head>
<style>
    body {{ padding: 24;
            background-color: {body_color};
         }}
    pre {{ background-color: rgb(226, 226, 226);
           margin: 0px;
           margin-top: 24px;
           padding: 16px;
        }}
    pre code {{ margin: 0px;
                padding: 0px;
             }}

    .center_content {{ width: 100%;
                       margin-right: auto; margin-left: auto;
                       text-align: center;
                       padding-top: 24; padding-bottom: 24;
                    }}
    .info_content {{ margin-bottom: 36;
                  }}
{head_css_style}
</style>
</head>

<body>
    <div class="top_content">
{top_content}
    </div>
    <div class="center_content">
        <img src="{graph_filename}.png" alt="{alt_text}" usemap="#{graph_name}">
{graph_map}
    </div>
    <div class="info_content">
{info_content}
    </div>
    <div class="bottom_content">
{bottom_content}
    </div>
</body>

</html>
"""


##
class ParamsDict():

    def __init__( self, params: dict = None ):
        self.params = params
        if self.params is None:
            self.params = {}

    def getDict(self):
        return self.params

    def get(self, key, default=None ):
        return self.params.get( key, default )

    def getValue(self, key, default=None ):
        key_value = self.params.get( key, None )
        if key_value is None:
            return default
        try:
            return key_value()
        except TypeError:
            pass
        return key_value

    def getNamed(self, key, name, default=None ):
        key_value = self.params.get( key, None )
        if key_value is None:
            return default
        try:
            return key_value( name )
        except TypeError:
            pass
        try:
            return key_value[ name ]
        except TypeError:
            pass
        return key_value


## ============================================================================


def set_node_html_attribs( graph, node_local_dir, filter_nodes=None ):
    local_dir = node_local_dir
    if len(local_dir) > 0:
        local_dir = local_dir + os.sep

    all_nodes = graph.getNodesAll()
    for node_obj in all_nodes:
        node_name = node_obj.get_name()
        raw_name  = unquote_name( node_name )
        if filter_nodes is not None:
            if raw_name not in filter_nodes:
                continue
        node_label = get_node_label( node_obj )
        node_obj.set( "tooltip", "node: " + node_label )
        node_filename = prepare_filesystem_name( raw_name )
        node_url = local_dir + node_filename + ".html"
        node_obj.set( "href", node_url )
