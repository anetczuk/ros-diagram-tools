## rosdiagramtools.py --help
```
usage: rosdiagramtools.py [-h] [-la] [--listtools]
                          {codedistribution,packagexmltree,classifynodes,buildtime,rosnodegraph,rostopicgraph,rosbagflow,rosverify}
                          ...

ROS diagram tools

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --listtools           List tools

subcommands:
  use one of tools

  {codedistribution,packagexmltree,classifynodes,buildtime,rosnodegraph,rostopicgraph,rosbagflow,rosverify}
                        one of tools
    codedistribution    source code distribution over packages
    packagexmltree      packages graph based on dependencies in package.xml
    classifynodes       match nodes to packages
    buildtime           catkin build time
    rosnodegraph        rosnode connection graph
    rostopicgraph       rostopic connection graph
    rosbagflow          generate sequence diagram based on messages from
                        rosbag
    rosverify           verify ROS packages
```



## rosdiagramtools.py codedistribution --help
```
usage: rosdiagramtools.py codedistribution [-h] [-la] [--clocpath CLOCPATH]
                                           [--highlight HIGHLIGHT]
                                           [--outraw OUTRAW] [--outpng OUTPNG]

source code distribution over packages

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --clocpath CLOCPATH   Path to file with dumped 'cloc' results
  --highlight HIGHLIGHT
                        List with items to highlight
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
```



## rosdiagramtools.py packagexmltree --help
```
usage: rosdiagramtools.py packagexmltree [-h] [-la] [-f FILE]
                                         [--nodeshape NODESHAPE]
                                         [--outraw OUTRAW] [--outpng OUTPNG]
                                         [--outhtml] [--outdir OUTDIR]

catkin packages graph

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  -f FILE, --file FILE  Read 'catkin list' output from file
  --nodeshape NODESHAPE
                        Shape of node: 'box', 'octagon' or other value
                        supprted by GraphViz dot
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## rosdiagramtools.py classifynodes --help
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



## rosdiagramtools.py buildtime --help
```
usage: rosdiagramtools.py buildtime [-h] [-la] [-f FILE] [-st SCALESECSTEP]
                                    [-sp SCALEPXNUM] [--outhtml]
                                    [--outdir OUTDIR]

catkin build schedule

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  -f FILE, --file FILE  Read catkin build log file
  -st SCALESECSTEP, --scalesecstep SCALESECSTEP
                        Scale time step. Timeline will be divided into steps
                        of given size.
  -sp SCALEPXNUM, --scalepxnum SCALEPXNUM
                        Scale pixel number - width in pixels of each timeline
                        step.
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## rosdiagramtools.py rosnodegraph --help
```
usage: rosdiagramtools.py rosnodegraph [-h] [-la]
                                       [--nodesdumppath NODESDUMPPATH]
                                       [--topicsdumppath TOPICSDUMPPATH]
                                       [--msgsdumppath MSGSDUMPPATH]
                                       [--servicesdumppath SERVICESDUMPPATH]
                                       [--srvsdumppath SRVSDUMPPATH] [-mfg]
                                       [-iri] [--outraw OUTRAW]
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
  -mfg, --mainfullgraph
                        Generate main full graph instead of compact one
  -iri, --includerosinternals
                        Include ROS internal items like /rosout and /record_*
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## rosdiagramtools.py rostopicgraph --help
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



## rosdiagramtools.py rosbagflow --help
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



## rosdiagramtools.py rosverify --help
```
usage: rosdiagramtools.py rosverify [-h] [-w WORKSPACE]

package overlay detector

optional arguments:
  -h, --help            show this help message and exit
  -w WORKSPACE, --workspace WORKSPACE
                        Workspace directory to analyze
```
