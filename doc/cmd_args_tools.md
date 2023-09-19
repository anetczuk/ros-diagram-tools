## <a name="main_help"></a> rosdiagramtools.py --help
```
usage: rosdiagramtools.py [-h] [-la] [--listtools]
                          {codedistribution,packagetree,classifynodes,buildtime,maketime,rosparamlist,rosnodegraph,rostopicgraph,rosindex,rosgeneral,rosbagflow,rosverify}
                          ...

ROS diagram tools

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --listtools           List tools

subcommands:
  use one of tools

  {codedistribution,packagetree,classifynodes,buildtime,maketime,rosparamlist,rosnodegraph,rostopicgraph,rosindex,rosgeneral,rosbagflow,rosverify}
                        one of tools
    codedistribution    source code distribution over packages
    packagetree         packages graph
    classifynodes       match nodes to packages
    buildtime           catkin build time
    maketime            objects compilation time based on make output
    rosparamlist        rosparam parameters list
    rosnodegraph        rosnode connection graph
    rostopicgraph       rostopic connection graph
    rosindex            index of diagrams
    rosgeneral          generate diagrams from provided data
    rosbagflow          generate sequence diagram based on messages from
                        rosbag
    rosverify           verify ROS packages
```



## <a name="codedistribution_help"></a> rosdiagramtools.py codedistribution --help
```
usage: rosdiagramtools.py codedistribution [-h] [-la]
                                           [--clocjsonpath CLOCJSONPATH]
                                           [--clocdumpdir CLOCDUMPDIR]
                                           [--filteritems FILTERITEMS]
                                           [--highlight HIGHLIGHT]
                                           [--outraw OUTRAW] [--outpng OUTPNG]
                                           [--outdir OUTDIR]

Source code distribution over packages. Tool can be feed with JSON or with
path to output of dumpclocpack tool.

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --clocjsonpath CLOCJSONPATH
                        Path to JSON file with dumped 'cloc' results
  --clocdumpdir CLOCDUMPDIR
                        Path to directory with dumped 'cloc' results
  --filteritems FILTERITEMS
                        File with list of items to filter
  --highlight HIGHLIGHT
                        List with items to highlight
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outdir OUTDIR       Output HTML
```



## <a name="packagetree_help"></a> rosdiagramtools.py packagetree --help
```
usage: rosdiagramtools.py packagetree [-h] [-la]
                                      [--catkinlistfile CATKINLISTFILE]
                                      [--packdumppath PACKDUMPPATH]
                                      [--classifynodesfile CLASSIFYNODESFILE]
                                      [--nodeshape NODESHAPE]
                                      [--topitems TOPITEMS]
                                      [--highlightitems HIGHLIGHTITEMS]
                                      [--descriptionjson DESCRIPTIONJSON]
                                      [--outraw OUTRAW] [--outpng OUTPNG]
                                      [--outhtml] [--outdir OUTDIR]

Packages graph. Tool can be feed with catkin putput (based on package.xml) or
with rospack output.

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --catkinlistfile CATKINLISTFILE
                        Read 'catkin list' data from file
  --packdumppath PACKDUMPPATH
                        Path to directory containing dumped 'rospack' output
  --classifynodesfile CLASSIFYNODESFILE
                        Nodes classification input file
  --nodeshape NODESHAPE
                        Shape of node: 'box', 'octagon' or other value
                        supprted by GraphViz dot
  --topitems TOPITEMS   File with list of items to filter on top
  --highlightitems HIGHLIGHTITEMS
                        File with list of items to highlight
  --descriptionjson DESCRIPTIONJSON
                        Path to JSON file with items description
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## <a name="classifynodes_help"></a> rosdiagramtools.py classifynodes --help
```
usage: rosdiagramtools.py classifynodes [-h] [-la] --packdumppath PACKDUMPPATH
                                        --launchdumppath LAUNCHDUMPPATH
                                        [--outfile OUTFILE]

match nodes to packages

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --packdumppath PACKDUMPPATH
                        Path to file containing dumped 'rospack' output
  --launchdumppath LAUNCHDUMPPATH
                        Path fo directory containing dumped 'roslaunch' output
  --outfile OUTFILE     Path to output file
```



## <a name="buildtime_help"></a> rosdiagramtools.py buildtime --help
```
usage: rosdiagramtools.py buildtime [-h] [-la] --buildlogfile BUILDLOGFILE
                                    [-st SCALESECSTEP] [-sp SCALEPXNUM]
                                    [--outhtml] [--outdir OUTDIR]

catkin build schedule

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --buildlogfile BUILDLOGFILE
                        Path to catkin build log file
  -st SCALESECSTEP, --scalesecstep SCALESECSTEP
                        Scale time step. Timeline will be divided into steps
                        of given size.
  -sp SCALEPXNUM, --scalepxnum SCALEPXNUM
                        Scale pixel number - width in pixels of each timeline
                        step.
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## <a name="maketime_help"></a> rosdiagramtools.py maketime --help
```
usage: rosdiagramtools.py maketime [-h] [-la] -clf COMPILELOGFILE
                                   [--outfile OUTFILE]

objects compilation time based on make output

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  -clf COMPILELOGFILE, --compilelogfile COMPILELOGFILE
                        Path to make compile log file
  --outfile OUTFILE     Path to output file
```



## <a name="rosparamlist_help"></a> rosdiagramtools.py rosparamlist --help
```
usage: rosdiagramtools.py rosparamlist [-h] [-la] --dumpyamlfile DUMPYAMLFILE
                                       [--outdir OUTDIR]

rosparam parameters list

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --dumpyamlfile DUMPYAMLFILE
                        Path to rosparam dump file
  --outdir OUTDIR       Output HTML
```



## <a name="rosnodegraph_help"></a> rosdiagramtools.py rosnodegraph --help
```
usage: rosdiagramtools.py rosnodegraph [-h] [-la]
                                       [--nodesdumppath NODESDUMPPATH]
                                       [--topicsdumppath TOPICSDUMPPATH]
                                       [--msgsdumppath MSGSDUMPPATH]
                                       [--servicesdumppath SERVICESDUMPPATH]
                                       [--srvsdumppath SRVSDUMPPATH]
                                       [--classifynodesfile CLASSIFYNODESFILE]
                                       [--highlightitems HIGHLIGHTITEMS]
                                       [--descriptionjson DESCRIPTIONJSON]
                                       [-mfg] [-iri] [--outraw OUTRAW]
                                       [--outpng OUTPNG] [--outhtml]
                                       [--outdir OUTDIR]

rosnode connection graph

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --nodesdumppath NODESDUMPPATH
                        Path to directory containing dumped 'rosnode' output
  --topicsdumppath TOPICSDUMPPATH
                        Path to directory containing dumped 'rostopic' output
  --msgsdumppath MSGSDUMPPATH
                        Path to directory containing dumped 'rosmsg' output
  --servicesdumppath SERVICESDUMPPATH
                        Path to directory containing dumped 'rosservice'
                        output
  --srvsdumppath SRVSDUMPPATH
                        Path to directory containing dumped 'rossrv' output
  --classifynodesfile CLASSIFYNODESFILE
                        Nodes classification input file
  --highlightitems HIGHLIGHTITEMS
                        File with list of items to highlight
  --descriptionjson DESCRIPTIONJSON
                        Path to JSON file with items description
  -mfg, --mainfullgraph
                        Generate main full graph instead of compact one
  -iri, --includerosinternals
                        Include ROS internal items like /rosout and /record_*
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## <a name="rostopicgraph_help"></a> rosdiagramtools.py rostopicgraph --help
```
usage: rosdiagramtools.py rostopicgraph [-h] [-la]
                                        [--topicsdumppath TOPICSDUMPPATH]
                                        [--outraw OUTRAW] [--outpng OUTPNG]

rostopic flow graph (tool is obsolete, use rosnodegraph)

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --topicsdumppath TOPICSDUMPPATH
                        Path to directory containing dumped 'rostopic list'
                        output
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
```



## <a name="rosindex_help"></a> rosdiagramtools.py rosindex --help
```
usage: rosdiagramtools.py rosindex [-h] [-la] [--packagesview PACKAGESVIEW]
                                   [--paramsview PARAMSVIEW]
                                   [--nodesview NODESVIEW]
                                   [--topicsview TOPICSVIEW]
                                   [--customlist [CUSTOMLIST [CUSTOMLIST ...]]]
                                   [--outdir OUTDIR]

index of diagrams

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --packagesview PACKAGESVIEW
                        Path to packages view
  --paramsview PARAMSVIEW
                        Path to params view
  --nodesview NODESVIEW
                        Path to nodes view
  --topicsview TOPICSVIEW
                        Path to topics view
  --customlist [CUSTOMLIST [CUSTOMLIST ...]]
                        Space-separated list of titles and links
  --outdir OUTDIR       Output HTML
```



## <a name="rosgeneral_help"></a> rosdiagramtools.py rosgeneral --help
```
usage: rosdiagramtools.py rosgeneral [-h] [-la] [--dumprootdir DUMPROOTDIR]
                                     [--launchdumppath LAUNCHDUMPPATH]
                                     [--classifynodesfile CLASSIFYNODESFILE]
                                     [--descriptionjsonfile DESCRIPTIONJSONFILE]
                                     [--pkgsfilterlist PKGSFILTERLIST] [-iri]
                                     [--outdir OUTDIR]

index of diagrams

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --dumprootdir DUMPROOTDIR
                        Path directory with standard dump directories
  --launchdumppath LAUNCHDUMPPATH
                        Path fo directory containing dumped 'roslaunch' output
  --classifynodesfile CLASSIFYNODESFILE
                        Nodes classification input file
  --descriptionjsonfile DESCRIPTIONJSONFILE
                        Path to JSON file with items description
  --pkgsfilterlist PKGSFILTERLIST
                        Path to file with list of packages to filter
  -iri, --includerosinternals
                        Include ROS internal items like /rosout and /record_*
  --outdir OUTDIR       Output HTML
```



## <a name="rosbagflow_help"></a> rosdiagramtools.py rosbagflow --help
```
usage: rosdiagramtools.py rosbagflow [-h] [-la] --rosbagpath ROSBAGPATH
                                     [--topicsdumppath TOPICSDUMPPATH]
                                     [--groupcalls] [--grouptopics]
                                     [--groupsubs] [--detectloops]
                                     [--writemessages]
                                     [--excludelistpath EXCLUDELISTPATH]
                                     [--outdir OUTDIR]

rosbag sequence diagram

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --rosbagpath ROSBAGPATH
                        Path to rosbag file
  --topicsdumppath TOPICSDUMPPATH
                        Path to directory containing dumped 'rostopic' output
  --groupcalls          Group calls to same topic
  --grouptopics         Group multiple topics in one call
  --groupsubs           Group topic's subscribers in one UML group
  --detectloops         Detect message loops and group in one UML loop
  --writemessages       Write message subpages
  --excludelistpath EXCLUDELISTPATH
                        Exclude list path
  --outdir OUTDIR       Output HTML
```



## <a name="rosverify_help"></a> rosdiagramtools.py rosverify --help
```
usage: rosdiagramtools.py rosverify [-h] [-w WORKSPACE]

package overlay detector

optional arguments:
  -h, --help            show this help message and exit
  -w WORKSPACE, --workspace WORKSPACE
                        Workspace directory to analyze
```
