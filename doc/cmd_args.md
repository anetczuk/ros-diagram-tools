## rosdiagramtools.py --help
```
usage: rosdiagramtools.py [-h] [-la] [--listtools]
                          {codedistribution,catkintree,classifynodes,catkinschedule,rosnodetree,rostopictree,rosbagflow,rosverify}
                          ...

ROS diagram tools

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --listtools           List tools

subcommands:
  use one of tools

  {codedistribution,catkintree,classifynodes,catkinschedule,rosnodetree,rostopictree,rosbagflow,rosverify}
                        one of tools
    codedistribution    source code distribution over packages
    catkintree          catkin packages graph
    classifynodes       match nodes to packages
    catkinschedule      catkin build schedule
    rosnodetree         rosnode connection graph
    rostopictree        rostopic connection graph
    rosbagflow          generate sequence diagram based on messages from
                        rosbag
    rosverify           verify ROS packages
```



## rosdiagramtools.py codedistribution --help
```
usage: rosdiagramtools.py codedistribution [-h] [-la] [--cloc_path CLOC_PATH]
                                           [--highlight HIGHLIGHT]
                                           [--outraw OUTRAW] [--outpng OUTPNG]

source code distribution over packages

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --cloc_path CLOC_PATH
                        File with 'cloc' results
  --highlight HIGHLIGHT
                        List with items to highlight
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
```



## rosdiagramtools.py catkintree --help
```
usage: rosdiagramtools.py catkintree [-h] [-la] [-f FILE]
                                     [--node_shape NODE_SHAPE]
                                     [--outraw OUTRAW] [--outpng OUTPNG]
                                     [--outhtml] [--outdir OUTDIR]

catkin packages graph

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  -f FILE, --file FILE  Read 'catkin list' output from file
  --node_shape NODE_SHAPE
                        Graph RAW output
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## rosdiagramtools.py classifynodes --help
```
usage: rosdiagramtools.py classifynodes [-h] [-la] --pack_list_file
                                        PACK_LIST_FILE --launch_dump_dir
                                        LAUNCH_DUMP_DIR [--out_file OUT_FILE]

match nodes to packages

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --pack_list_file PACK_LIST_FILE
                        Dump file containing 'rospack' output
  --launch_dump_dir LAUNCH_DUMP_DIR
                        Dump directory containing 'roslaunch' output data
  --out_file OUT_FILE   Output map file
```



## rosdiagramtools.py catkinschedule --help
```
usage: rosdiagramtools.py catkinschedule [-h] [-la] [-f FILE]
                                         [-st SCALE_SEC_STEP]
                                         [-sp SCALE_PX_NUM] [--outhtml]
                                         [--outdir OUTDIR]

catkin build schedule

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  -f FILE, --file FILE  Read catkin build log file
  -st SCALE_SEC_STEP, --scale_sec_step SCALE_SEC_STEP
                        Scale time step
  -sp SCALE_PX_NUM, --scale_px_num SCALE_PX_NUM
                        Scale pixel number
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## rosdiagramtools.py rosnodetree --help
```
usage: rosdiagramtools.py rosnodetree [-h] [-la] [--dump_dir DUMP_DIR]
                                      [--topics_dump_dir TOPICS_DUMP_DIR]
                                      [--msgs_dump_dir MSGS_DUMP_DIR]
                                      [--services_dump_dir SERVICES_DUMP_DIR]
                                      [--srvs_dump_dir SRVS_DUMP_DIR]
                                      [--outraw OUTRAW] [--outpng OUTPNG]
                                      [--outhtml] [--outdir OUTDIR]

rosnode connection graph

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --dump_dir DUMP_DIR   Dump directory containing 'rosnode' output data
  --topics_dump_dir TOPICS_DUMP_DIR
                        Dump directory containing 'rostopic' output data
  --msgs_dump_dir MSGS_DUMP_DIR
                        Dump directory containing 'rosmsg' output data
  --services_dump_dir SERVICES_DUMP_DIR
                        Dump directory containing 'rosservice' output data
  --srvs_dump_dir SRVS_DUMP_DIR
                        Dump directory containing 'rossrv' output data
  --outraw OUTRAW       Graph RAW output
  --outpng OUTPNG       Graph PNG output
  --outhtml             Output HTML
  --outdir OUTDIR       Output HTML
```



## rosdiagramtools.py rostopictree --help
```
usage: rosdiagramtools.py rostopictree [-h] [-la] [--dump_dir DUMP_DIR]
                                       [--outraw OUTRAW] [--outpng OUTPNG]

rostopic flow graph

optional arguments:
  -h, --help           show this help message and exit
  -la, --logall        Log all messages
  --dump_dir DUMP_DIR  Dump directory containing 'rostopic list' output data
  --outraw OUTRAW      Graph RAW output
  --outpng OUTPNG      Graph PNG output
```



## rosdiagramtools.py rosbagflow --help
```
usage: rosdiagramtools.py rosbagflow [-h] [-la] --bag_path BAG_PATH
                                     [--topic_dump_dir TOPIC_DUMP_DIR]
                                     [--outdir OUTDIR] [--group_calls]
                                     [--group_topics] [--group_subs]
                                     [--detect_loops] [--write_messages]
                                     [--exclude_list_path EXCLUDE_LIST_PATH]

rosbag sequence diagram

optional arguments:
  -h, --help            show this help message and exit
  -la, --logall         Log all messages
  --bag_path BAG_PATH   Path to rosbag file
  --topic_dump_dir TOPIC_DUMP_DIR
                        Dump directory containing 'rostopic' output data
  --outdir OUTDIR       Output HTML
  --group_calls         Group calls to same topic
  --group_topics        Group multiple topics in one call
  --group_subs          Group topic's subscribers in one UML group
  --detect_loops        Detect message loops and group in one UML loop
  --write_messages      Write message subpages
  --exclude_list_path EXCLUDE_LIST_PATH
                        Exclude list path
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
